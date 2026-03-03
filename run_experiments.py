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

    # TODO: We could also add parameter conditions (planning_sigma, alpha_epistemic, n_clusters) to the CSV for better analysis later on.
    
    def __init__(self, trial_name):
        super().__init__('exp_logger')
        self.trial_name = trial_name
        self.csv_file = open(f"{trial_name}.csv", mode='w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['step', 'position_error', 'rotational_error', 'shannon_entropy', 'spatial_entropy','convergence_gmm', 'epistemic', 'pragmatic', 'selected_action'
        'actual real x',
        'actual real y',
        'actual real yaw',
        'x weighted mean cluster ',
        'y weighted mean cluster ',
        'yaw weighted mean cluster',
        'std x weighted',
        'std y weighted',
        ])

        self.metrics = None
        self.current_step = 0
        self.finished = False
        self.status = "PENDING"

        # Subscribers
        self.create_subscription(String, '/trial_status', self.status_callback, 10)
        self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

    def metrics_callback(self, msg):
        """
        TODO: Check with core.py publish_metrics() for exact order and content.
        """
        self.metrics = msg.data
        self.log_step()

    def status_callback(self, msg):
        if "SUCCESS" in msg.data or "FAILURE" in msg.data:
            self.status = msg.data
            self.finished = True

    def log_step(self):
        if self.metrics:
            self.writer.writerow([
                self.current_step, 
                self.metrics[7] if len(self.metrics) > 7 else -1.0,  # position error from AIC metrics
                self.metrics[8] if len(self.metrics) > 8 else -1.0,  # rotational error from AIC metrics
                self.metrics[9] if len(self.metrics) > 9 else -1.0,  # shannon entropy from AIC metrics
                self.metrics[10] if len(self.metrics) > 10 else -1.0, # spatial entropy from AIC metrics
                self.metrics[6] if len(self.metrics) > 6 else -1.0,  # convergence gmm from AIC metrics
                self.metrics[0] if len(self.metrics) > 0 else -1.0,  # epistemic
                self.metrics[1] if len(self.metrics) > 1 else -1.0,  # pragmatic
                self.metrics[11] if len(self.metrics) > 11 else -1.0,  # selected action
                self.metrics[12] if len(self.metrics) > 12 else -1.0,  # actual real x
                self.metrics[13] if len(self.metrics) > 13 else -1.0,  # actual real y
                self.metrics[14] if len(self.metrics) > 14 else -1.0,  # actual real yaw
                self.metrics[15] if len(self.metrics) > 15 else -1.0,  # x weighted mean cluster 
                self.metrics[16] if len(self.metrics) > 16 else -1.0,  # y weighted mean cluster 
                self.metrics[17] if len(self.metrics) > 17 else -1.0,  # yaw weighted mean cluster
                self.metrics[18] if len(self.metrics) > 18 else -1.0,  # std x weighted
                self.metrics[19] if len(self.metrics) > 19 else -1.0,  # std y weighted
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
        'starting_poses_1000.csv' # 100 or 1000
    ) 
    poses = load_poses_from_csv(poses_file_path)

    algos = ["active_inf"]#, "random_walk", "entropy_min"]  
    
    summary_f = open('summary_results.csv', mode='w')
    summary_writer = csv.writer(summary_f)
    summary_writer.writerow(['algorithm', 'pose_index', 'status', 'steps', 'alpha', 'beta'])
    
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
                f"algo_mode:={algo}", f"spawn_x:={p['x']}", f"spawn_y:={p['y']}", f"spawn_yaw:={p['yaw']}"
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
               
            summary_writer.writerow([algo, i, logger.status, logger.current_step, 
                                     logger.metrics[3] if logger.metrics and len(logger.metrics) > 3 else -1.0,  # alpha
                                     logger.metrics[4] if logger.metrics and len(logger.metrics) > 4 else -1.0])  # beta
            summary_f.flush()

            logger.csv_file.close()
            cleanup_processes(sim_proc, ctrl_proc)
            
            print(f"✓ Finished {trial_id}: {logger.status} (steps={logger.current_step})")
            print(f"  CSV saved: {trial_id}.csv")

    summary_f.close()
    rclpy.shutdown()
    print(f"\n{'='*60}")
    print("✓ All experiments completed!")
    print(f"Summary saved to: summary_results.csv")
    print(f"{'='*60}")
    
if __name__ == '__main__':
    run_benchmarking()