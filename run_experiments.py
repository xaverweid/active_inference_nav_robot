"""Orchestrator" for running all your experiments in a systematic way. This script will automate the process of launching your ROS 2 nodes with different Active Inference variants and starting positions.
    The Workflow
    Generate/Load Poses: Read your 50 (x,y,Y) coordinates from a .csv file.
    Outer Loop: Iterate through your Control variants (AIC, random walk, etc.))
    Inner Loop: Iterate through the 50 poses.
    The Launch: Use the Python subprocess module to trigger your ROS 2 launch commands.
    The Monitor: Wait for a specific "Success" message on a ROS topic or a timeout.
    The Cleanup: Kill everything to ensure a "clean slate" for the next run (avoiding memory leaks or leftover Gazebo state).
    Data Logging: Save the results of each run (success/failure, time taken, etc.) to a CSV for later analysis.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
import subprocess, signal, os, time, csv, numpy as np

class ExperimentLogger(Node):
    def __init__(self, trial_name):
        super().__init__('exp_logger')
        self.trial_name = trial_name
        self.csv_file = open(f"{trial_name}.csv", mode='w')
        self.writer = csv.writer(self.csv_file)
        # Header: Step, GT_X, GT_Y, AMCL_X, AMCL_Y, Error, Entropy, Cumulative_Dist
        self.writer.writerow(['step', 'gt_x', 'gt_y', 'amcl_x', 'amcl_y', 'error', 'convergence', 'cum_dist'])

        # State Variables
        self.gt_pose = None
        self.amcl_pose = None
        self.last_gt_pose = None
        self.cumulative_distance = 0.0
        self.metrics = 0.0
        self.current_step = 0
        self.finished = False
        self.status = "PENDING"

        # Subscribers
        # Note: Gazebo Harmonic publishes GT to /model/<robot_name>/odometry via bridge
        self.create_subscription(Odometry, '/model/diff_bot/odometry', self.gt_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(String, '/trial_status', self.status_callback, 10)
        # THE TRIGGER: Every time this receives a message, we log a row.
        self.create_subscription(Float32MultiArray, '/aic_metrics', self.metrics_callback, 10)

    def gt_callback(self, msg):
        self.gt_pose = msg.pose.pose.position
        if self.last_gt_pose:
            # Calculate Path Efficiency (Cumulative Distance)
            d = np.sqrt((self.gt_pose.x - self.last_gt_pose.x)**2 + (self.gt_pose.y - self.last_gt_pose.y)**2)
            self.cumulative_distance += d
        self.last_gt_pose = self.gt_pose

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose.position

    def metrics_callback(self, msg):
        """
        Triggered by your AIC algorithm after a step.
        msg.data is expected to be [entropy, other_metric1, ...]
        """
        self.metrics = msg.data
        self.log_step()

    def status_callback(self, msg):
        if "SUCCESS" in msg.data or "FAILURE" in msg.data:
            self.status = msg.data
            self.finished = True

    def log_step(self):
        if self.gt_pose and self.amcl_pose:
            error = np.sqrt((self.gt_pose.x - self.amcl_pose.x)**2 + (self.gt_pose.y - self.amcl_pose.y)**2)
            
            self.writer.writerow([
                self.current_step, 
                self.gt_pose.x, 
                self.gt_pose.y, 
                self.amcl_pose.x, 
                self.amcl_pose.y, 
                error, 
                self.metrics[6],  # This is Convergence, incorporate shannon entropy later!
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
    print("Cleaning up processes...")
    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    
    # Force kill Gazebo Harmonic components
    subprocess.run(["pkill", "-9", "gz-sim-server"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "gz-sim-gui"], stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)
    time.sleep(3)

def run_benchmarking():
    rclpy.init()
    # Poses and Algorithms setup 
    poses_file_path = 'starting_poses.csv'  # Path to your CSV file with starting poses
    poses = load_poses_from_csv(poses_file_path)

    algos = ["active_inf", "passive_amcl", "random_walk", "classical_amcl", "standard_dwa"]  # Add your control variants here 
    
    for algo in algos:
        for i, p in enumerate(poses):

            # 1. Setup the individual experimental run
            trial_id = f"trial_{algo}_p{i}"
            print(f"\n>>> Starting {trial_id}")

            logger = ExperimentLogger(trial_id)

            # 2. Launch simulation and AIC
            sim_proc = subprocess.Popen([
                    "ros2", "launch", "diff_drive_robot", "robot_launch.py",
                    f"x_pose:={p['x']}", f"y_pose:={p['y']}", f"yaw_pose:={p['yaw']}"
                ], preexec_fn=os.setsid)
            
            time.sleep(12) # Wait for Gazebo and AMCL to stabilize
            
            ctrl_proc = subprocess.Popen([
                    "ros2", "launch", "active_inference_loc", "aic_launch.py",
                    f"algo_mode:={algo}"
                ], preexec_fn=os.setsid)
            
            # 3. Monitor
            # Spin until AIC sends SUCCESS/FAILURE or TIMEOUT
            start_t = time.time()
            timeout = 300  # 5 minutes timeout for each run
            while rclpy.ok() and not logger.finished:
                    rclpy.spin_once(logger, timeout_sec=0.1)
                    if (time.time() - start_t) > timeout:
                        logger.status = "TIMEOUT"
                        break

            final_error = -1.0
            if logger.gt_pose and logger.amcl_pose:
                final_error = np.sqrt((logger.gt_pose.x - logger.amcl_pose.x)**2 + (logger.gt_pose.y - logger.amcl_pose.y)**2)
                
            summary_writer.writerow([algo, i, logger.status, logger.current_step, final_error, logger.cumulative_distance])
            summary_f.flush()

            # 5. Cleanup
            logger.csv_file.close()
            cleanup_processes(sim_proc, ctrl_proc)
            print(f"Finished {trial_id} with status: {logger.status}")

    rclpy.shutdown()
    
if __name__ == '__main__':
    run_benchmarking()