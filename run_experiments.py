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
import os
from datetime import datetime
import signal
import sys

class ExperimentLogger(Node):
    """Subscribes to ROS topics and logs experiment data to CSV."""
    
    def __init__(self, trial_name):
        super().__init__('exp_logger')
        self.trial_name = trial_name
        self.csv_file = open(f"{trial_name}.csv", mode='w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['step', 'position_error', 'rotational_error', 'shannon_entropy', 'spatial_entropy','convergence_gmm', 'epistemic', 'pragmatic', 'selected_action',
        'actual real x',
        'actual real y',
        'actual real yaw',
        'x weighted mean cluster ',
        'y weighted mean cluster ',
        'yaw weighted mean cluster',
        'std x weighted',
        'std y weighted',
        'bimodal_score',
        'is_bimodal', 
        'peak1_x', 'peak1_y', 'peak1_yaw',
        'peak2_x', 'peak2_y', 'peak2_yaw',
        'peak_distance',
        'is_wait_streak_reset',
        'num_particles',
        'gazebo_position_belief_x',
        'gazebo_position_belief_y',
        'gazebo_rotation_belief',
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
                self.metrics[20] if len(self.metrics) > 20 else -1.0,  # bimodal score
                self.metrics[21] if len(self.metrics) > 21 else -1.0,  # is bimodal (boolean)
                self.metrics[22] if len(self.metrics) > 22 else -1.0,  # peak1_x
                self.metrics[23] if len(self.metrics) > 23 else -1.0,  # peak1_y
                self.metrics[24] if len(self.metrics) > 24 else -1.0,  # peak1_yaw
                self.metrics[25] if len(self.metrics) > 25 else -1.0,  # peak2_x
                self.metrics[26] if len(self.metrics) > 26 else -1.0,  # peak2_y
                self.metrics[27] if len(self.metrics) > 27 else -1.0,  # peak2_yaw
                self.metrics[28] if len(self.metrics) > 28 else -1.0,  # peak_distance
                self.metrics[33] if len(self.metrics) > 33 else -1.0,  # is_wait_streak_reset
                self.metrics[34] if len(self.metrics) > 34 else -1.0,  # num_particles
                self.metrics[35] if len(self.metrics) > 35 else -1.0, # gazebo_position_belief x
                self.metrics[36] if len(self.metrics) > 36 else -1.0, # gazebo_position_belief y
                self.metrics[37] if len(self.metrics) > 37 else -1.0, # gazebo_rotation_belief
            ])
            self.current_step += 1
            self.csv_file.flush()

def get_clean_env():
    """
    Filters LD_LIBRARY_PATH to prevent Gazebo from trying to load 
    incompatible libraries from Snap folders.
    """
    env = os.environ.copy()
    ld_path = env.get("LD_LIBRARY_PATH", "")
    
    if ld_path:
        # Split the path, keep only parts that DON'T contain 'snap'
        parts = ld_path.split(":")
        cleaned_parts = [p for p in parts if "snap" not in p.lower()]
        env["LD_LIBRARY_PATH"] = ":".join(cleaned_parts)
        
    return env

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
    print("Cleaning up system processes...")
    
    # 1. Graceful SIGTERM first
    for proc in [sim_proc, ctrl_proc]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(2)
    
    # 2. Force kill anything remaining
    for proc in [sim_proc, ctrl_proc]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    # 3. Kill ALL relevant ROS/Gazebo processes by name
    for pattern in ["gz", "ruby", "gzserver", "gzclient", 
                    "robot_state_publisher", "amcl", 
                    "nav2", "aic_node", "diff_drive"]:
        subprocess.run(["pkill", "-9", "-f", pattern], stderr=subprocess.DEVNULL)
    
    # 4. Clear shared memory (Gazebo uses this heavily)
    subprocess.run("ipcs -m | awk 'NR>3 {print $2}' | xargs -r ipcrm -m", 
                   shell=True, stderr=subprocess.DEVNULL)
    
    # 5. Restart ROS2 daemon
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)
    time.sleep(1)
    subprocess.run(["ros2", "daemon", "start"], stderr=subprocess.DEVNULL)
    
    # 6. Longer wait — give OS time to release ports/sockets
    time.sleep(8)  # was 5

active_sim_proc = None
active_ctrl_proc = None

def handle_interrupt(sig, frame):
    print("\n[INTERRUPTED] Ctrl+C caught — cleaning up...")
    if active_sim_proc and active_ctrl_proc:
        cleanup_processes(active_sim_proc, active_ctrl_proc)
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_interrupt)
signal.signal(signal.SIGTERM, handle_interrupt)

RUN_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")

def run_benchmarking():
    global active_sim_proc, active_ctrl_proc
    rclpy.init()
    clean_env = get_clean_env()
    
    
    """
    PARAMETERS:
    Make sure to have the robot_launch.py configured with the correct world file and map file that correspond to your experiment (h_map vs my_map) for both
    .sdf and .yaml files in robot_launch.py 

    """
    poses_file_path = os.path.join(
        get_package_share_directory('diff_drive_robot'),
        'config',
        'starting_poses_1000_h_map.csv' # [starting_poses_1000_h_map.csv, starting_poses_1000_my_map.csv]
    ) 
    poses = load_poses_from_csv(poses_file_path)

    algos = ["entropy_min"]  
    seconds_per_step = ['1', '5'] #, '1', '5'
    map_name= 'h_map' # 'h_map', 'my_map'

    data_root = os.path.join(os.getcwd(), "src", "data")

    for algo in algos:
        for seconds in seconds_per_step:
            algo_dir = os.path.join(data_root, map_name, seconds, algo)
            os.makedirs(algo_dir, exist_ok=True)
            
            summary_filename = os.path.join(algo_dir, f"summary_{map_name}_{seconds}s_{algo}_{RUN_TIMESTAMP}.csv")
            summary_f = open(summary_filename, mode='w')
            summary_writer = csv.writer(summary_f)
            summary_writer.writerow(['algorithm', 'pose_index', 'status', 'steps', 'alpha', 'beta',
                                    'convergence_threshold', 'bimodal_score_threshold',
                                    'planning_sigma', 'spatial_entropy_res'])

            for i, p in enumerate(poses[0:200], start=0):

                # Hard reset every 100 runs: sleep longer to let system breathe
                if i > 0 and i % 100 == 0:
                    print(f"[PERIODIC RESET] Run {i} — extended cooldown...")
                    time.sleep(30)  # let OS fully settle

                trial_id = os.path.join(algo_dir, f"trial_{map_name}_{seconds}s_{algo}_p{i+1:04d}_{RUN_TIMESTAMP}")
                print(f"\n{'='*60}")
                print(f">>> Starting {trial_id}")
                print(f"{'='*60}")

                logger = ExperimentLogger(trial_id)

                # [1/3] Launching robot simulator
                print(f"[1/3] Launching robot simulator at pose ({p['x']}, {p['y']})...")
                sim_proc = subprocess.Popen([
                    "ros2", "launch", "diff_drive_robot", "robot_launch.py",
                    f"x_pose:={p['x']}", f"y_pose:={p['y']}", f"yaw_pose:={p['yaw']}"
                ], preexec_fn=os.setsid, env=clean_env) # Pass clean_env here

                active_sim_proc = sim_proc

                # Wait for simulator to fully initialize and AMCL to warm up
                print("[2/3] Waiting 12 seconds for initialization...")
                time.sleep(12)
                
                # [3/3] Launching AIC controller
                print(f"[3/3] Launching AIC node in mode: {algo}_{seconds}s")
                ctrl_proc = subprocess.Popen([
                    "ros2", "launch", "active_inference_loc", "aic_launch.py", 
                    "enable_viz:=false", # disable Belief Monitor Node
                    f"algo_mode:={algo}", 
                    f"spawn_x:={p['x']}", f"spawn_y:={p['y']}", f"spawn_yaw:={p['yaw']}",
                    f"seconds_per_step:={seconds}"
                ], preexec_fn=os.setsid, env=clean_env) # Pass clean_env here
                
                active_ctrl_proc = ctrl_proc

                # Run trial with timeout
                start_t = time.time()
                timeout = 420  # 7 minutes per trial maximum
                while rclpy.ok() and not logger.finished:
                    rclpy.spin_once(logger, timeout_sec=0.1)
                    elapsed = time.time() - start_t
                    if elapsed > timeout:
                        logger.status = "TIMEOUT"
                        print(f"[TIMEOUT] Trial exceeded {timeout}s")
                        break

                # Calculate final metrics
                
                summary_writer.writerow([algo, i+1, logger.status, logger.current_step, 
                                        logger.metrics[3] if logger.metrics and len(logger.metrics) > 3 else -1.0,  # alpha
                                        logger.metrics[4] if logger.metrics and len(logger.metrics) > 4 else -1.0,  # beta
                                        logger.metrics[29] if logger.metrics and len(logger.metrics) > 29 else -1.0, # convergence_threshold            
                                        logger.metrics[30] if logger.metrics and len(logger.metrics) > 30 else -1.0, # bimodal_score_threshold       
                                        logger.metrics[31] if logger.metrics and len(logger.metrics) > 31 else -1.0, # planning_sigma           
                                        logger.metrics[32] if logger.metrics and len(logger.metrics) > 32 else -1.0, # spatial_entropy_res
                ])
                summary_f.flush()

                logger.csv_file.close()
                logger.destroy_node()
                cleanup_processes(sim_proc, ctrl_proc)
                
                print(f"✓ Finished {trial_id}: {logger.status} (steps={logger.current_step})")
                print(f"  CSV saved: {trial_id}.csv")

            summary_f.close()

    rclpy.shutdown()
    print(f"\n{'='*60}")
    print("✓ All experiments completed!")
    print(f"Summary saved to: summary_results_{RUN_TIMESTAMP}.csv")
    print(f"{'='*60}")
    
if __name__ == '__main__':
    run_benchmarking()