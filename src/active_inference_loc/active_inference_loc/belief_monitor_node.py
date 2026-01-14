import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from sklearn.cluster import KMeans
from nav2_msgs.msg import ParticleCloud

# Importing your existing utilities
from .utils import get_map_metadata, ParticleClusturer, get_covariance_ellipse

class BeliefMonitorNode(Node):
    def __init__(self):
        super().__init__('belief_monitor_node')

        # 1. Initialize Logic
        self.clusturer = ParticleClusturer(n_clusters=5)
        self.map_data = None
        self.current_metrics = [0.0, 0.0, 0.0] # [Epistemic, Pragmatic, Total G]

        # 2. QoS for Map
        map_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        # 3. Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.cloud_sub = self.create_subscription(ParticleCloud, '/particle_cloud', self.cloud_callback, 10)
        self.metrics_sub = self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

        # 4. Matplotlib Setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('Active Inference Belief & Performance Monitor')
        
        # Dashboard Text Box (Upper Left)
        self.dashboard = self.ax.text(0.02, 0.98, 'Waiting for data...', transform=self.ax.transAxes, 
                                      verticalalignment='top', family='monospace', fontsize=9,
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))

        self.get_logger().info("Belief Monitor Node with AIC Dashboard Started.")

    def metrics_callback(self, msg):
        """ Receives [Epistemic, Pragmatic, Total_G] from the aic_node. """
        if len(msg.data) >= 3:
            self.current_metrics = msg.data

    def map_callback(self, msg):
        if self.map_data is None:
            self.map_data = get_map_metadata(msg)
            self.ax.imshow(self.map_data['data'], cmap='gray', origin='lower',
                           extent=[self.map_data['origin_x'], 
                                   self.map_data['origin_x'] + self.map_data['width'] * self.map_data['resolution'],
                                   self.map_data['origin_y'], 
                                   self.map_data['origin_y'] + self.map_data['height'] * self.map_data['resolution']],
                           alpha=0.6)
            self.ax.set_title("Robot Posterior Belief & AIC Metrics")

    def cloud_callback(self, msg):
        if self.map_data is None: return

        # --- A. DATA PROCESSING ---
        # Use the 4D converter
        # raw points needs to be 4D for clustering
        raw_points, raw_weights = self.clusturer.cloud_to_numpy(msg)
        n_particles = len(raw_points)
        
        # 1. Clustering for Brain & Metrics (using 4D)
        # representative_poses: List of (x, y, cos(theta), sin(theta))
        representative_poses, cluster_weights = self.clusturer.get_representative_clusters_from_np(raw_points, raw_weights)

        # 2. Uncertainty Calculations
        shannon_h = -np.sum([w * np.log(w + 1e-9) for w in cluster_weights])
        sum_sq_w = np.sum(raw_weights**2)
        ess_val = 1.0 / (sum_sq_w + 1e-9)
        diversity_ratio = (ess_val / n_particles) * 100

        # --- B. VISUALIZATION ---
        for artist in self.ax.collections + self.ax.patches + self.ax.texts:
            if artist != self.dashboard: artist.remove()

        # Plot Raw Particles (X and Y only)
        self.ax.scatter(raw_points[:, 0], raw_points[:, 1], s=1, c='blue', alpha=0.1)

        # Re-run K-means just for the labels to draw ellipses correctly
        # Note: We use raw_points[:, :2] if you want spatial ellipses only,
        # or raw_points if you want orientation to influence cluster shapes.
        kmeans_viz = KMeans(n_clusters=len(representative_poses), n_init=10)
        labels = kmeans_viz.fit_predict(raw_points) 

        # Unpacking 4D into 3D for visualization
        for i, (mx, my, ctheta, stheta) in enumerate(representative_poses):
            mtheta = np.arctan2(stheta, ctheta)
            cluster_mask = (labels == i)
            if np.sum(cluster_mask) < 3: continue
            
            # Pass only X and Y to the ellipse function
            res = get_covariance_ellipse(raw_points[cluster_mask, :2])
            if res:
                width, height, angle = res
                color = plt.cm.plasma(i / self.clusturer.n_clusters)
                
                ell = Ellipse(
                    xy=(mx, my), 
                    width=width, 
                    height=height, 
                    angle=angle, 
                    edgecolor=color, 
                    fc=color, 
                    lw=2, 
                    alpha=0.3
                )
                self.ax.add_patch(ell)
                
                # Draw a small "heading" line to show the cluster's yaw
                # Use ctheta and stheta for direction
                dx = 0.2 * ctheta
                dy = 0.2 * stheta           
                self.ax.arrow(mx, my, dx, dy, color='red', head_width=0.05)

        # --- C. DASHBOARD UPDATE ---
        # (Surprise logic remains same)
        current_surprise = -np.log(np.max(cluster_weights) + 1e-9) 
        
        table_text = (
            f"▼ BELIEF METRICS\n"
            f"Shannon Entropy (H):  {shannon_h:.3f} nats\n"
            f"Filter Diversity:     {diversity_ratio:.1f}%\n"
            f"Variational F:        {current_surprise:.2f}\n"
            f"-------------------------------\n"
            f"▼ AIC POLICY (G)\n"
            f"Expected Epistemic:   {self.current_metrics[0]:.2f}\n"
            f"Expected Pragmatic:   {self.current_metrics[1]:.2f}\n"
            f"Total Expected G:     {self.current_metrics[2]:.2f}"
        )
        self.dashboard.set_text(table_text)
        self.fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = BeliefMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()