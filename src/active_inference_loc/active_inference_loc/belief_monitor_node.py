import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from nav2_msgs.msg import ParticleCloud
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from sklearn.cluster import KMeans
import threading

# Importing your existing utilities
from .utils import get_map_metadata, ParticleClusturer, get_covariance_ellipse

class BeliefMonitorNode(Node):
    def __init__(self):
        super().__init__('belief_monitor_node')

        self.clusturer = ParticleClusturer(n_clusters=5)
        self.map_metadata = None
        self.current_metrics = [0.0, 0.0, 0.0]
        self.latest_cloud_data = None
        self.lock = threading.Lock()

        map_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        particle_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.cloud_sub = self.create_subscription(ParticleCloud, '/particle_cloud', self.cloud_callback, particle_qos)
        self.metrics_sub = self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('AIC Belief Monitor')
        
        self.dashboard = self.ax.text(0.02, 0.98, 'Waiting for data...', transform=self.ax.transAxes, 
                                      verticalalignment='top', family='monospace', fontsize=9,
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'),
                                      zorder=100)
        
        self.get_logger().info("Belief Monitor Node Started")

    def metrics_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 3:
                self.current_metrics = msg.data

    def map_callback(self, msg):
        if self.map_metadata is None:
            self.map_metadata = get_map_metadata(msg)
            
            # --- ROTATE MAP DATA 270 CW (equivalent to -90 CW) ---
            rotated_map = np.rot90(self.map_metadata['data'], k=3)
            
            # --- CALCULATE NEW EXTENT (Correct for 270 CW) ---
            # Transformation: x' = -y, y' = x
            # New X range is negative old Y range: [-y_max, -y_min]
            # New Y range is old X range: [x_min, x_max]
            orig_x_min = self.map_metadata['origin_x']
            orig_x_max = orig_x_min + self.map_metadata['width'] * self.map_metadata['resolution']
            orig_y_min = self.map_metadata['origin_y']
            orig_y_max = orig_y_min + self.map_metadata['height'] * self.map_metadata['resolution']
            
            self.rotated_extent = [-orig_y_max, -orig_y_min, orig_x_min, orig_x_max]

            self.ax.imshow(rotated_map, cmap='gray', origin='lower',
                           extent=self.rotated_extent, alpha=0.6)
            self.ax.set_title("Robot Posterior Belief")

    def cloud_callback(self, msg):
        with self.lock:
            self.latest_cloud_data = msg

    def update_plot(self):
        cloud_msg = None
        with self.lock:
            cloud_msg = self.latest_cloud_data
            self.latest_cloud_data = None 
            metrics = self.current_metrics

        if cloud_msg is None or self.map_metadata is None:
            return

        try:
            raw_points, raw_weights = self.clusturer.cloud_to_numpy(cloud_msg)
            if len(raw_points) == 0: return

            rotated_points = np.zeros_like(raw_points)
            
            # Map coordinates rotation: 270 CW (equivalent to -90 CW)
            # New X = -Old Y
            # New Y = Old X
            rotated_points[:, 0] = -raw_points[:, 1]  
            rotated_points[:, 1] = raw_points[:, 0] 
            
            # Heading rotation: 270 CW
            # New Cos = -Old Sin
            # New Sin = Old Cos
            rotated_points[:, 2] = -raw_points[:, 3]
            rotated_points[:, 3] = raw_points[:, 2]

            # Now that points are rotated, we cluster them
            representative_poses, cluster_weights = self.clusturer.get_representative_clusters_from_np(rotated_points, raw_weights)
            
            shannon_h = -np.sum([w * np.log(w + 1e-9) for w in cluster_weights])
            ess_val = 1.0 / (np.sum(raw_weights**2) + 1e-9)
            diversity_ratio = (ess_val / len(raw_points)) * 100

            # --- 3. RENDERING ---
            for artist in list(self.ax.collections) + list(self.ax.patches):
                artist.remove()
            for txt in list(self.ax.texts):
                if txt != self.dashboard: txt.remove()

            # Plot Rotated Particles
            self.ax.scatter(rotated_points[:, 0], rotated_points[:, 1], s=1, c='blue', alpha=0.1)
            # Assign labels for the rotated clusters
            kmeans_viz = KMeans(n_clusters=len(representative_poses), n_init=5)
            labels = kmeans_viz.fit_predict(rotated_points[:, :2]) 

            for i, (mx, my, ctheta, stheta) in enumerate(representative_poses):
                cluster_mask = (labels == i)
                if np.sum(cluster_mask) < 3: continue
                
                # Get ellipse for the rotated cluster
                res = get_covariance_ellipse(rotated_points[cluster_mask, :2])
                if res:
                    w, h, ang = res
                    color = plt.cm.plasma(i / self.clusturer.n_clusters)
                    ell = Ellipse(xy=(mx, my), width=w, height=h, angle=ang, 
                                  edgecolor=color, fc=color, lw=2, alpha=0.3, zorder=3)
                    self.ax.add_patch(ell)
                    # Arrow points correctly in rotated space
                    self.ax.arrow(mx, my, 0.4*ctheta, 0.4*stheta, color='red', head_width=0.1, zorder=5)
            # Dashboard Update
            current_surprise = -np.log(np.max(cluster_weights) + 1e-9) 
            table_text = (
                f"▼ BELIEF METRICS \n"
                f"Shannon Entropy (H):  {shannon_h:.3f} nats\n"
                f"Filter Diversity:     {diversity_ratio:.1f}%\n"
                f"Variational F:        {current_surprise:.2f}\n"
                f"-------------------------------\n"
                f"▼ AIC POLICY (G)\n"
                f"Expected Epistemic:   {metrics[0]:.2f}\n"
                f"Expected Pragmatic:   {metrics[1]:.2f}\n"
                f"Total Expected G:     {metrics[2]:.2f}"
            )
            self.dashboard.set_text(table_text)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        except Exception as e:
            self.get_logger().error(f"Visualization Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BeliefMonitorNode()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.update_plot()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()