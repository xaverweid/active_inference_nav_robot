"""
Orchestrates experimental runs across multiple algorithms and starting poses.
Logs trajectory and convergence metrics to CSV.
Relies on /aic_metrics topic 
"""
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
import subprocess, signal, os, time, csv, numpy as np

class ExperimentLogger(Node):
    """Subscribes to ROS topics and logs experiment data to CSV."""
    
    def __init__(self, trial_name):
        super().__init__('exp_logger')
        self.trial_name = trial_name
        self.csv_file = open(f"{trial_name}.csv", mode='w')
        self.writer = csv.writer(self.csv_file)
        # Header includes position error (from metrics[7])
        self.writer.writerow(['step', 'gt_x', 'gt_y', 'amcl_x', 'amcl_y', 'error', 'convergence', 'epistemic', 'pragmatic', 'cum_dist'])

        # State
        self.gt_pose = None
        self.amcl_pose = None
        self.last_gt_pose = None
        self.cumulative_distance = 0.0
        self.metrics = None
        self.current_step = 0
        self.finished = False
        self.status = "PENDING"

        # Subscribers
        self.create_subscription(Odometry, '/ground_truth/pose', self.gt_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(String, '/trial_status', self.status_callback, 10)
        self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

    def gt_callback(self, msg):
        self.gt_pose = msg.pose.pose.position
        if self.last_gt_pose:
            d = np.sqrt((self.gt_pose.x - self.last_gt_pose.x)**2 + (self.gt_pose.y - self.last_gt_pose.y)**2)
            self.cumulative_distance += d
        self.last_gt_pose = self.gt_pose

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose.position

    def metrics_callback(self, msg):
        """
        msg.data: [epistemic, pragmatic, total_efe, alpha, beta, runtime, convergence, position_error]
        """
        self.metrics = msg.data
        self.log_step()

    def status_callback(self, msg):
        if "SUCCESS" in msg.data or "FAILURE" in msg.data:
            self.status = msg.data
            self.finished = True

    def log_step(self):
        if self.gt_pose and self.amcl_pose and self.metrics:
            error = np.sqrt((self.gt_pose.x - self.amcl_pose.x)**2 + (self.gt_pose.y - self.amcl_pose.y)**2)
            
            self.writer.writerow([
                self.current_step, 
                self.gt_pose.x, 
                self.gt_pose.y, 
                self.amcl_pose.x, 
                self.amcl_pose.y, 
                error,
                self.metrics[6],  # convergence
                self.metrics[0],  # epistemic
                self.metrics[1],  # pragmatic
                self.cumulative_distance
            ])
            self.current_step += 1
            self.csv_file.flush()

def load_poses_from_csv(poses_file_path):
    poses = []
    with open(poses_file_path, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            poses.append({
                'x': float(row['x']), 
                'y': float(row['y']), 
                'yaw': float(row['yaw'])
            })
    return poses

def cleanup_processes(sim_proc, ctrl_proc):
    """Kill all ROS processes cleanly."""
    print("Cleaning up...")
    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    
    subprocess.run(["pkill", "-9", "gz-sim-server"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "gz-sim-gui"], stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)
    time.sleep(3)

def run_benchmarking():
    rclpy.init()
    
    poses_file_path = os.path.join(
        get_package_share_directory('diff_drive_robot'),
        'config',
        'starting_poses.csv'
    ) 
    poses = load_poses_from_csv(poses_file_path)

    algos = ["active_inf", "passive_amcl", "random_walk", "classical_amcl", "standard_dwa"]
    
    summary_f = open('summary_results.csv', mode='w')
    summary_writer = csv.writer(summary_f)
    summary_writer.writerow(['algorithm', 'pose_index', 'status', 'steps', 'final_error', 'cumulative_distance'])
    
    for algo in algos:
        for i, p in enumerate(poses):
            trial_id = f"trial_{algo}_p{i}"
            print(f"\n{'='*60}")
            print(f">>> Starting {trial_id}")
            print(f"{'='*60}")

            logger = ExperimentLogger(trial_id)

            # **FIXED: Launch robot first with position and map**
            print(f"[1/3] Launching robot simulator at pose ({p['x']}, {p['y']}, {p['yaw']})...")
            sim_proc = subprocess.Popen([
                "ros2", "launch", "diff_drive_robot", "robot_launch.py",
                f"x_pose:={p['x']}", f"y_pose:={p['y']}", f"yaw_pose:={p['yaw']}"
            ], preexec_fn=os.setsid)
            
            # Wait for simulator to fully initialize and AMCL to warm up
            print("[2/3] Waiting 12 seconds for simulator and AMCL to initialize...")
            time.sleep(12)
            
            # **FIXED: Launch AIC controller WITH algo_mode parameter**
            print(f"[3/3] Launching AIC node in mode: {algo}")
            ctrl_proc = subprocess.Popen([
                "ros2", "launch", "active_inference_loc", "aic_launch.py",
                f"algo_mode:={algo}"
            ], preexec_fn=os.setsid)
            
            # Run trial with timeout
            start_t = time.time()
            timeout = 300  # 5 minutes per trial
            while rclpy.ok() and not logger.finished:
                rclpy.spin_once(logger, timeout_sec=0.1)
                elapsed = time.time() - start_t
                if elapsed > timeout:
                    logger.status = "TIMEOUT"
                    print(f"[TIMEOUT] Trial exceeded {timeout}s")
                    break

            # Calculate final metrics
            final_error = -1.0
            if logger.gt_pose and logger.amcl_pose:
                final_error = np.sqrt((logger.gt_pose.x - logger.amcl_pose.x)**2 + (logger.gt_pose.y - logger.amcl_pose.y)**2)
                
            summary_writer.writerow([algo, i, logger.status, logger.current_step, final_error, logger.cumulative_distance])
            summary_f.flush()

            logger.csv_file.close()
            cleanup_processes(sim_proc, ctrl_proc)
            
            print(f"✓ Finished {trial_id}: {logger.status} (steps={logger.current_step}, error={final_error:.3f}m)")
            print(f"  CSV saved: {trial_id}.csv")

    summary_f.close()
    rclpy.shutdown()
    print(f"\n{'='*60}")
    print("✓ All experiments completed!")
    print(f"Summary saved to: summary_results.csv")
    print(f"{'='*60}")
    
if __name__ == '__main__':
    run_benchmarking()