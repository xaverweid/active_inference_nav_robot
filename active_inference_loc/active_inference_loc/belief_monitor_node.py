import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
import threading
import scipy.special

# Assuming these utilities are available in your workspace
from .utils import get_map_metadata, ParticleClusturer, get_covariance_ellipse

class BeliefMonitorNode(Node):
    def __init__(self):
        super().__init__('belief_monitor_node')

        self.clusturer = ParticleClusturer()
        self.map_metadata = None
        self.current_metrics = None
        self.latest_cloud_data = None
        self.latest_weights = None
        self.lock = threading.Lock()

        self.action_id_to_name = {
                0.0: 'WAIT',
                1.0: 'FORWARD_SMALL',
                2.0: 'FORWARD_LARGE',
                3.0: 'ROTATE_LEFT',
                4.0: 'ROTATE_RIGHT',
                5.0: 'TURN_LEFT',
                6.0: 'TURN_RIGHT',
                7.0: 'BACKWARD_SMALL'
            }

        map_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        particle_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.cloud_sub = self.create_subscription(Float32MultiArray, '/belief/particles_filtered', self.cloud_callback, particle_qos)
        self.metrics_sub = self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

        plt.ion()
        # Create two subplots: [Map] [Dashboard]
        self.fig, (self.ax, self.ax_info) = plt.subplots(1, 2, figsize=(10, 8), gridspec_kw={'width_ratios': [2, 1]})
        plt.subplots_adjust(left=0.05, right=0.95, wspace=0.00)
        self.fig.canvas.manager.set_window_title('AIC Belief Monitor')
        
        # Setup info axis (hide borders and ticks)
        self.ax_info.axis('off')
        self.dashboard = self.ax_info.text(0.05, 0.95, 'Waiting for data...', 
                                           verticalalignment='top', family='monospace', fontsize=10,
                                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.1, edgecolor='none'))
        
        self.get_logger().info("Belief Monitor Node Started with 90 deg rotation")

    def map_callback(self, msg):
        if self.map_metadata is None:
            self.map_metadata = get_map_metadata(msg)
            
            # Rotate map 90 degrees CCW (left)
            map_data = np.rot90(self.map_metadata['data'], k=3)
            
            orig_x_min = self.map_metadata['origin_x']
            orig_x_max = orig_x_min + self.map_metadata['width'] * self.map_metadata['resolution']
            orig_y_min = self.map_metadata['origin_y']
            orig_y_max = orig_y_min + self.map_metadata['height'] * self.map_metadata['resolution']
            
            # Coordinate Transform for Extent: x' = -y, y' = x
            # New X range: [-orig_y_max, -orig_y_min]
            # New Y range: [orig_x_min, orig_x_max]
            self.extent = [-orig_y_max, -orig_y_min, orig_x_min, orig_x_max]

            self.ax.clear()
            self.ax.imshow(map_data, cmap='gray', origin='lower', extent=self.extent, alpha=0.6)
            self.ax.set_title("Robot Belief Monitor")

    def metrics_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 3:
                self.current_metrics = msg.data

    def cloud_callback(self, msg):
        try:
            raw_data = np.array(msg.data).reshape(-1, 5)
            # Original raw data
            points = raw_data[:, :2]           
            cos_yaw = raw_data[:, 2]
            sin_yaw = raw_data[:, 3]
            weights = raw_data[:, 4]
            
            # Apply 90-degree CCW transformation: x' = -y, y' = x
            rot_points = np.column_stack((-points[:, 1], points[:, 0]))
            
            # Rotate vectors: [cos', sin'] = [-sin, cos]
            rot_cos = -sin_yaw
            rot_sin = cos_yaw

            self.latest_cloud_data = np.column_stack((rot_points, rot_cos, rot_sin))
            self.latest_weights = weights

        except Exception as e:
            self.get_logger().error(f"Failed to reshape particle data: {e}")

    def update_plot(self):
        with self.lock:
            if self.current_metrics is None or self.latest_cloud_data is None or self.latest_weights is None:
                return
            points_rot = self.latest_cloud_data.copy()
            weights = self.latest_weights.copy()
            metrics = list(self.current_metrics)
            self.latest_cloud_data = None
            self.latest_weights = None

        try:
            # GMM Clustering on rotated points
            cluster_poses, cluster_weights, _ = self.clusturer.get_representative_clusters_from_gmm(points_rot, weights)
            
            # Clear previous frame artists
            for artist in list(self.ax.collections) + list(self.ax.patches):
                artist.remove()

            # Plot Rotated Particles
            self.ax.scatter(points_rot[:, 0], points_rot[:, 1], s=1, c='blue', alpha=0.1)  
            
            labels = self.clusturer.gmm.predict(points_rot)
            norm = plt.Normalize(vmin=0, vmax=np.max(cluster_weights))
            cmap = plt.cm.RdYlGn
            
            for i, (mx, my, ctheta, stheta) in enumerate(cluster_poses):
                weight = cluster_weights[i]
                cluster_mask = (labels == i)
                if np.sum(cluster_mask) < 3: continue
                
                color = cmap(norm(weight))
                res = get_covariance_ellipse(points_rot[cluster_mask, :2])
                if res:
                    width, height, angle = res
                    # Draw Belief Ellipse (angle is already relative to rotated points)
                    ell = Ellipse(xy=(mx, my), width=width, height=height, angle=angle, 
                                edgecolor=color, fc=color, lw=2, alpha=0.4, zorder=3)
                    self.ax.add_patch(ell)
                    
                    # Draw Heading Arrow (already transformed in cloud_callback)
                    self.ax.arrow(mx, my, 0.4*ctheta, 0.4*stheta, color='blue', 
                                head_width=0.1, zorder=5, alpha=min(1.0, weight*5))
                
            # Dashboard Update
            shannon_h = f"{metrics[9]:.2f}" if len(metrics) > 9 else 'N/A'
            spatial_entropy = f"{metrics[10]:.2f}" if len(metrics) > 10 else 'N/A'
            pos_error = f"{metrics[7]:.3f}" if len(metrics) > 7 else 'N/A'
            rot_error = f"{metrics[8]:.3f}" if len(metrics) > 8 else 'N/A'
            action_val = metrics[11] if len(metrics) > 11 else 'N/A'
            action_string = self.action_id_to_name.get(float(action_val), "UNKNOWN")            
            
            table_text = (
                f"▼ GLOBAL PARAMETERS\n"
                f"alpha (epistemic):  {metrics[3]:.2f}\n"
                f"beta  (pragmatic):  {metrics[4]:.2f}\n"
                f"-------------------------------\n"
                f"▼ BELIEF METRICS on all P\n"
                f"Shannon H:    {shannon_h}\n"
                f"Spatial H:    {spatial_entropy}\n"
                f"Convergence:  {metrics[6]:.2f}\n"
                f"-------------------------------\n"
                f"▼ AIC POLICY (G)\n"
                f"Exp. Epistemic: {metrics[0]:.2f}\n"
                f"Exp. Pragmatic: {metrics[1]:.2f}\n"
                f"Total G:        {metrics[2]:.2f}\n"
                f"-------------------------------\n"
                f"▼ STATUS\n"
                f"Runtime:        {int(metrics[5]) if len(metrics) > 5 else 'N/A'}\n"
                f"Particles:      {int(metrics[34]) if len(metrics) > 34 else 'N/A'}\n"
                f"Pos Error:      {pos_error}m\n"
                f"Rot Error:      {rot_error}rad\n"
                f"Action:         {action_string}\n"
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
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()