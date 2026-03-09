"""
Heatmap Analysis for Active Inference Localization Experiments
Produces:
  1. Visited pose heatmap (where the robot physically was)
  2. Gaze/direction heatmap (where the algorithm wanted to move)
  3. Quiver overlay (mean intended direction per map cell)
All plots split by pre/post convergence.

Usage:
  python analysis/analyze_heatmaps.py

Expected project layout:
  <repo_root>/
    active_inference_nav_robot/
      diff_drive_robot/
        maps/
          my_map.pgm
    analysis/
      analyze_heatmaps.py   <- this file
    results/                <- CSV files (git-ignored)
    figures/                <- output figures (git-ignored)
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.ndimage import gaussian_filter
import yaml

def load_map_metadata(map_path):
    """Read resolution and origin from the .yaml file next to the .pgm."""
    yaml_path = os.path.splitext(map_path)[0] + '.yaml'
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Map yaml not found at '{yaml_path}'")
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    return {
        'resolution': meta['resolution'],
        'origin_x':   meta['origin'][0],
        'origin_y':   meta['origin'][1],
    }

# ─────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)          # active_inference_nav_robot/
PARENT_ROOT = os.path.dirname(REPO_ROOT)           

sim_id = "numba-5" # adjust to run numba-5, numba-500, numba-5-3
CSV_DIR = os.path.join(PARENT_ROOT, "results", sim_id) # ../results/ — outside repo
OUTPUT_DIR = os.path.join(PARENT_ROOT, "figures", sim_id)  # ../figures/ — outside repo
MAP_PATH   = os.path.join(REPO_ROOT, "diff_drive_robot", "maps", "my_map.pgm") 
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─────────────────────────────────────────────
# MAP METADATA
# ─────────────────────────────────────────────

meta = load_map_metadata(MAP_PATH)
MAP_RESOLUTION = meta['resolution']
MAP_ORIGIN_X   = meta['origin_x']
MAP_ORIGIN_Y   = meta['origin_y']

# ─────────────────────────────────────────────
# ACTION ENCODING
# action_float = enumerate(actions_dict.keys()) index
# ─────────────────────────────────────────────

ACTION_MAP = {
    0.0: 'WAIT',
    1.0: 'FORWARD_SMALL',
    2.0: 'FORWARD_LARGE',
    3.0: 'ROTATE_LEFT',
    4.0: 'ROTATE_RIGHT',
    5.0: 'TURN_LEFT',
    6.0: 'TURN_RIGHT',
    7.0: 'BACKWARD_SMALL',
}

# Make sure these are aligned with active_inference_nav_robot/active_inference_loc/active_inference_loc/utils.py
ACTION_PARAMS = {
    'WAIT':           {'linear': 0.0,   'angular': 0.0},
    'FORWARD_SMALL':  {'linear': 0.15,  'angular': 0.0},
    'FORWARD_LARGE':  {'linear': 0.30,  'angular': 0.0},
    'ROTATE_LEFT':    {'linear': 0.0,   'angular': 0.4},
    'ROTATE_RIGHT':   {'linear': 0.0,   'angular': -0.4},
    'TURN_LEFT':      {'linear': 0.15,  'angular': 0.3},
    'TURN_RIGHT':     {'linear': 0.15,  'angular': -0.3},
    'BACKWARD_SMALL': {'linear': -0.15, 'angular': 0.0},
}

NO_TRANSLATION        = {'WAIT', 'ROTATE_LEFT', 'ROTATE_RIGHT'}
ACTION_DT             = 0.9
CONVERGENCE_THRESHOLD = 0.2
BIN_SIZE_M            = 0.3
QUIVER_GRID_M         = 0.5
GAUSSIAN_SIGMA        = 1.5 # Increase it if the result looks noisy, decrease it if it looks too blurry.

POS_ERROR_THRESHOLD = 0.5   # metres
ROT_ERROR_THRESHOLD = 0.5   # radians

# ─────────────────────────────────────────────
# LOAD CSVs
# ─────────────────────────────────────────────

# Since i had some script error and had the last 7 headers missing, i did add them manually. 
COLUMN_NAMES = [
    'step', 'position_error', 'rotational_error', 'shannon_entropy', 'spatial_entropy',
    'convergence_gmm', 'epistemic', 'pragmatic', 'selected_action',
    'actual_real_x', 'actual_real_y', 'actual_real_yaw',
    'x_weighted_mean_cluster', 'y_weighted_mean_cluster', 'yaw_weighted_mean_cluster',
    'std_x_weighted', 'std_y_weighted',
    'bimodal_score', 'is_bimodal',
    'peak1_x', 'peak1_y', 'peak1_yaw',
    'peak2_x', 'peak2_y', 'peak2_yaw',
    'peak_distance'
]

def load_all_csvs(csv_dir):
    files = sorted(glob.glob(os.path.join(csv_dir, "*.csv")))
    files = [f for f in files if not os.path.basename(f).startswith("summary_")]

    if not files:
        raise FileNotFoundError(
            f"No CSV files found in '{csv_dir}'.\n"
            f"Expected path: {csv_dir}"
        )
    print(f"Found {len(files)} CSV files")

    dfs = []
    for i, f in enumerate(files):
        try:
            df = pd.read_csv(f,
                             skiprows=1,         # skip the broken header row
                             names=COLUMN_NAMES) # apply correct column names
            df['run_id'] = i
            dfs.append(df)
        except Exception as e:
            print(f"  Skipping {os.path.basename(f)}: {e}")

    combined = pd.concat(dfs, ignore_index=True)
    combined = combined.apply(pd.to_numeric, errors='coerce') 
    print(f"Total rows loaded: {len(combined):,}")
    return combined

def load_valid_summary(csv_dir):
    """Load summary file, excluding runs that collided at step 0."""
    summary_files = [f for f in os.listdir(csv_dir) if f.startswith('summary_')]
    if not summary_files:
        raise FileNotFoundError("No summary file found.")
    summary = pd.read_csv(os.path.join(csv_dir, summary_files[0]))
    # Exclude instant collisions (step=0) — bad spawn positions, not representative
    valid = summary[~((summary['status'].str.contains('Collision', na=False)) &
                      (summary['steps'] == 0))].copy()
    return valid
# ─────────────────────────────────────────────
# LOAD MAP
# ─────────────────────────────────────────────

def load_map(map_path):
    if not os.path.exists(map_path):
        print(f"  Warning: map not found at '{map_path}' — plotting without background.")
        return None
    img = plt.imread(map_path)
    if img.ndim == 3:
        img = img[:, :, 0]
    return img

# ─────────────────────────────────────────────
# PREPARE DATAFRAME
# ─────────────────────────────────────────────

def action_to_vector(yaw, action_name):
    p = ACTION_PARAMS.get(action_name)
    if p is None:
        return 0.0, 0.0
    lin, ang = p['linear'], p['angular']
    if abs(lin) < 1e-6:
        return 0.0, 0.0
    if abs(ang) < 1e-6:
        return lin * np.cos(yaw) * ACTION_DT, lin * np.sin(yaw) * ACTION_DT
    yaw_end = yaw + ang * ACTION_DT
    avg_yaw = (yaw + yaw_end) / 2.0
    return lin * np.cos(avg_yaw) * ACTION_DT, lin * np.sin(avg_yaw) * ACTION_DT


def prepare_df(df):
    df = df.copy()
    df['action_name'] = df['selected_action'].astype(float).map(ACTION_MAP).fillna('UNKNOWN')
    # removed converged column entirely
    vecs = df.apply(
        lambda r: action_to_vector(r['yaw_weighted_mean_cluster'], r['action_name']), axis=1
    )
    df['dx'] = [v[0] for v in vecs]
    df['dy'] = [v[1] for v in vecs]
    df = df.dropna(subset=['actual_real_x', 'actual_real_y', 'yaw_weighted_mean_cluster'])
    return df

# ─────────────────────────────────────────────
# PLOT HELPERS
# ─────────────────────────────────────────────

def get_map_extent(map_img):
    h, w = map_img.shape[:2]
    return [MAP_ORIGIN_X, MAP_ORIGIN_X + w * MAP_RESOLUTION,
            MAP_ORIGIN_Y, MAP_ORIGIN_Y + h * MAP_RESOLUTION]

def get_ranges(map_img, df):
    if map_img is not None:
        e = get_map_extent(map_img)
        return [e[0], e[1]], [e[2], e[3]]
    return ([df['actual_real_x'].min(), df['actual_real_x'].max()],
            [df['actual_real_y'].min(), df['actual_real_y'].max()])

def setup_axis(ax, map_img):
    if map_img is not None:
        ax.imshow(map_img, cmap='gray', origin='lower',
                  extent=get_map_extent(map_img), alpha=0.55, zorder=0)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

def hot_transparent():
    c = plt.cm.hot(np.linspace(0, 1, 256))
    c[:, -1] = np.linspace(0, 1, 256)
    return LinearSegmentedColormap.from_list('hot_alpha', c)

def build_heatmap(x, y, x_range, y_range):
    bx = max(1, int((x_range[1] - x_range[0]) / BIN_SIZE_M))
    by = max(1, int((y_range[1] - y_range[0]) / BIN_SIZE_M))
    h, _, _ = np.histogram2d(x, y, bins=[bx, by], range=[x_range, y_range])
    h = gaussian_filter(h.T, sigma=GAUSSIAN_SIGMA)
    h[h < 0.1] = np.nan
    return h

def draw_heatmap(ax, h, x_range, y_range, label):
    im = ax.imshow(h,
                   extent=[x_range[0], x_range[1], y_range[0], y_range[1]],
                   origin='lower', cmap=hot_transparent(),
                   vmin=0, vmax=np.nanpercentile(h, 98),
                   zorder=2, interpolation='bilinear')
    plt.colorbar(im, ax=ax, label=label, shrink=0.8)

# ─────────────────────────────────────────────
# ANALYSIS 0 — STARTING POSITIONS ON MAP
# ─────────────────────────────────────────────

def plot_starting_positions(df, map_img):
    starts = df[df['step'] == 0][['actual_real_x', 'actual_real_y', 'actual_real_yaw', 'run_id']].copy()

    fig, ax = plt.subplots(figsize=(10, 9))
    setup_axis(ax, map_img)

    # Dot at each start position
    ax.scatter(
        starts['actual_real_x'], starts['actual_real_y'],
        c='red', s=20, alpha=0.6, zorder=3, label=f'Start positions (n={len(starts)})'
    )

    # Arrow showing initial yaw direction
    arrow_len = 0.2  # metres — adjust if arrows are too long/short
    ax.quiver(
        starts['actual_real_x'],
        starts['actual_real_y'],
        arrow_len * np.cos(starts['actual_real_yaw']),
        arrow_len * np.sin(starts['actual_real_yaw']),
        color='red', alpha=0.5,
        scale=1, scale_units='xy',
        width=0.003, headwidth=4,
        zorder=4
    )

    ax.set_title(f'Starting Positions and Orientations Across All Runs\n({len(starts)} runs)',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper right')
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'starting_positions.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

# ─────────────────────────────────────────────
# ANALYSIS 1 — VISITED POSE HEATMAP
# ─────────────────────────────────────────────

def plot_position_heatmap(df, map_img):
    x_range, y_range = get_ranges(map_img, df)
    fig, ax = plt.subplots(1, 1, figsize=(8, 7))
    fig.suptitle('Visited Pose Heatmap — Where Does the Robot Go?',
                 fontsize=14, fontweight='bold')
    setup_axis(ax, map_img)
    h = build_heatmap(df['actual_real_x'], df['actual_real_y'], x_range, y_range)
    draw_heatmap(ax, h, x_range, y_range, 'Visit density')
    ax.set_title(f'All Steps\n({len(df):,} steps, {df["run_id"].nunique()} runs)')
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'heatmap_position.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

def plot_gaze_heatmap(df, map_img):
    x_range, y_range = get_ranges(map_img, df)
    trans = df[~df['action_name'].isin(NO_TRANSLATION)]
    fig, ax = plt.subplots(1, 1, figsize=(8, 7))
    fig.suptitle('Gaze Heatmap — Where Does the Algorithm Want to Move?',
                 fontsize=14, fontweight='bold')
    setup_axis(ax, map_img)
    h = build_heatmap(trans['actual_real_x'] + trans['dx'],
                      trans['actual_real_y'] + trans['dy'],
                      x_range, y_range)
    draw_heatmap(ax, h, x_range, y_range, 'Gaze density')
    ax.set_title(f'All Steps\n({len(trans):,} steps)')
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'heatmap_gaze.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

def plot_quiver_overlay(df, map_img):
    trans = df[~df['action_name'].isin(NO_TRANSLATION)].copy()
    fig, ax = plt.subplots(1, 1, figsize=(8, 7))
    fig.suptitle('Mean Intended Direction per Map Cell',
                 fontsize=14, fontweight='bold')
    setup_axis(ax, map_img)
    trans['cell_x'] = np.floor(trans['actual_real_x'] / QUIVER_GRID_M) * QUIVER_GRID_M
    trans['cell_y'] = np.floor(trans['actual_real_y'] / QUIVER_GRID_M) * QUIVER_GRID_M
    g = trans.groupby(['cell_x', 'cell_y'])[['dx', 'dy']].mean().reset_index()
    mag = np.sqrt(g['dx']**2 + g['dy']**2).replace(0, np.nan)
    u = g['dx'] / mag * QUIVER_GRID_M * 0.6
    v = g['dy'] / mag * QUIVER_GRID_M * 0.6
    ax.quiver(g['cell_x'] + QUIVER_GRID_M / 2,
              g['cell_y'] + QUIVER_GRID_M / 2,
              u, v, mag,
              cmap='plasma', alpha=0.85,
              scale=1, scale_units='xy',
              width=0.004, headwidth=4, zorder=3)
    ax.set_title(f'All Steps\n({len(trans):,} steps)')
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'quiver_direction.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

# ─────────────────────────────────────────────
# ANALYSIS 2 — COLLISIONS
# ─────────────────────────────────────────────

def save_collision_report(csv_dir, output_dir):
    summary_files = [f for f in os.listdir(csv_dir) if f.startswith('summary_')]
    if not summary_files:
        print("No summary file found.")
        return

    summary = pd.read_csv(os.path.join(csv_dir, summary_files[0]))

    collisions  = summary[summary['status'].str.contains('Collision', na=False)].copy()
    instant_col = collisions[collisions['steps'] == 0]
    mid_run_col = collisions[collisions['steps'] > 0]

    out = os.path.join(output_dir, 'collision_report.txt')
    with open(out, 'w') as f:

        f.write('='*60 + '\n')
        f.write('COLLISION REPORT\n')
        f.write('='*60 + '\n\n')

        f.write(f"Total runs          : {len(summary)}\n")
        f.write(f"Successful runs     : {summary['status'].str.contains('Convergence').sum()}\n")
        f.write(f"Total collisions    : {len(collisions)}\n")
        f.write(f"  At start (step=0) : {len(instant_col)}\n")
        f.write(f"  Mid-run (step>0)  : {len(mid_run_col)}\n\n")

        f.write('='*60 + '\n')
        f.write('MID-RUN COLLISIONS — investigate these\n')
        f.write('='*60 + '\n')
        if len(mid_run_col) == 0:
            f.write('None.\n\n')
        else:
            f.write(mid_run_col[['pose_index', 'status', 'steps']].to_string(index=False))
            f.write('\n\n')

        f.write('='*60 + '\n')
        f.write('COLLISIONS AT START (step=0) — bad spawn positions\n')
        f.write('='*60 + '\n')
        if len(instant_col) == 0:
            f.write('None.\n\n')
        else:
            f.write(instant_col[['pose_index', 'status', 'steps']].to_string(index=False))
            f.write('\n')

    print(f"Saved: {out}")

    if len(mid_run_col) > 0:
        poses_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "diff_drive_robot", "config", "starting_poses_1000.csv"
        )
        if os.path.exists(poses_file):
            poses = pd.read_csv(poses_file)
            mid_run_poses = poses.iloc[mid_run_col['pose_index'].values - 1]

            map_img = load_map(MAP_PATH)
            fig, ax = plt.subplots(figsize=(10, 9))
            setup_axis(ax, map_img)

            colors = plt.cm.tab10(np.linspace(0, 1, len(mid_run_col)))

            for color, (_, row) in zip(colors, mid_run_col.iterrows()):
                pose_idx  = int(row['pose_index'])
                n_steps   = int(row['steps'])

                # Find the matching CSV for this run
                run_files = glob.glob(os.path.join(csv_dir, f"*p{pose_idx:04d}*.csv"))
                if not run_files:
                    continue

                try:
                    run_df = pd.read_csv(run_files[0],
                                        skiprows=1,
                                        names=COLUMN_NAMES)
                    run_df = pd.to_numeric(run_df.stack(), errors='coerce').unstack()
                except Exception as e:
                    print(f"  Could not load run p{pose_idx}: {e}")
                    continue

                traj_x = run_df['actual_real_x'].values
                traj_y = run_df['actual_real_y'].values

                if len(traj_x) == 0:
                    continue

                start_x, start_y = traj_x[0],  traj_y[0]
                end_x,   end_y   = traj_x[-1], traj_y[-1]

                # Full trajectory line
                ax.plot(traj_x, traj_y,
                        color=color, linewidth=1.2, alpha=0.7, zorder=3)

                # Start — green circle
                ax.scatter(start_x, start_y,
                        color=color, s=60, marker='o',
                        edgecolors='green', linewidths=2, zorder=5)

                # End / collision — red X
                ax.scatter(end_x, end_y,
                        color=color, s=80, marker='X',
                        edgecolors='red', linewidths=2, zorder=5)

                # Label
                ax.annotate(f"p{pose_idx} ({n_steps}s)",
                            (end_x, end_y),
                            textcoords='offset points', xytext=(6, 4),
                            fontsize=7, color=color, fontweight='bold')

            # Legend entries
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], marker='o', color='w', markeredgecolor='green',
                    markersize=8, label='Start position'),
                Line2D([0], [0], marker='X', color='w', markeredgecolor='red',
                    markersize=8, label='Collision position'),
                Line2D([0], [0], color='gray', linewidth=1.2, label='Trajectory'),
            ]
            ax.legend(handles=legend_elements, loc='upper right')
            ax.set_title(f'Mid-Run Collision Trajectories (n={len(mid_run_col)})',
                        fontsize=13, fontweight='bold')
            plt.tight_layout()
            out_map = os.path.join(output_dir, 'collision_midrun_map.pdf')
            plt.savefig(out_map, dpi=150, bbox_inches='tight')
            print(f"Saved: {out_map}")
            plt.close()

# --- VALID POSITION RANGES from non-collision runs ---
    valid_x, valid_y = [], []
    for _, row in summary[~summary['status'].str.contains('Collision', na=False)].iterrows():
        pose_idx  = int(row['pose_index'])
        run_files = glob.glob(os.path.join(csv_dir, f"*p{pose_idx:04d}*.csv"))
        if not run_files:
            continue
        try:
            run_df = pd.read_csv(run_files[0], skiprows=1, names=COLUMN_NAMES)
            run_df = pd.to_numeric(run_df.stack(), errors='coerce').unstack()
            valid_x.extend(run_df['actual_real_x'].dropna().tolist())
            valid_y.extend(run_df['actual_real_y'].dropna().tolist())
        except Exception:
            continue

    valid_x = np.array(valid_x)
    valid_y = np.array(valid_y)

    # Append to txt report
    with open(out, 'a') as f:
        f.write('\n' + '='*60 + '\n')
        f.write('VALID (NON-COLLISION) POSITION RANGES\n')
        f.write('='*60 + '\n\n')
        f.write(f"  x: [{valid_x.min():.3f}, {valid_x.max():.3f}]\n")
        f.write(f"  y: [{valid_y.min():.3f}, {valid_y.max():.3f}]\n")
        f.write(f"  Total valid points: {len(valid_x):,}\n\n")
    print(f"  Valid position ranges appended to: {out}")

    # Plot valid positions on map
    map_img = load_map(MAP_PATH)
    fig, ax = plt.subplots(figsize=(10, 9))
    setup_axis(ax, map_img)

    ax.scatter(valid_x, valid_y, c='steelblue', s=1, alpha=0.15,
               zorder=2, label=f'Valid positions (n={len(valid_x):,})')

    # Mark the observed boundary extents
    ax.axvline(valid_x.min(), color='red', linestyle='--', linewidth=1, label=f'x min: {valid_x.min():.3f}')
    ax.axvline(valid_x.max(), color='red', linestyle='--', linewidth=1, label=f'x max: {valid_x.max():.3f}')
    ax.axhline(valid_y.min(), color='orange', linestyle='--', linewidth=1, label=f'y min: {valid_y.min():.3f}')
    ax.axhline(valid_y.max(), color='orange', linestyle='--', linewidth=1, label=f'y max: {valid_y.max():.3f}')

    ax.set_title('Valid (Non-Collision) Position Coverage', fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', markerscale=5)
    plt.tight_layout()
    out_valid = os.path.join(output_dir, 'valid_position_ranges.pdf')
    plt.savefig(out_valid, dpi=150, bbox_inches='tight')
    print(f"Saved: {out_valid}")
    plt.close()

        # --- NEW: All collision positions on map ---
    poses_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "diff_drive_robot", "config", "starting_poses_1000.csv"
    )
    if os.path.exists(poses_file) and len(collisions) > 0:
        poses = pd.read_csv(poses_file)

        map_img = load_map(MAP_PATH)
        fig, ax = plt.subplots(figsize=(10, 9))
        setup_axis(ax, map_img)

        # Instant collisions — bad spawn points
        if len(instant_col) > 0:
            instant_poses = poses.iloc[instant_col['pose_index'].values - 1]
            ax.scatter(
                instant_poses['x'], instant_poses['y'],
                c='orange', s=40, alpha=0.8, zorder=4,
                label=f'Spawn collision (n={len(instant_col)})'
            )

        # Mid-run collisions — plot the actual collision endpoint from CSV
        mid_run_end_x, mid_run_end_y = [], []
        for _, row in mid_run_col.iterrows():
            pose_idx = int(row['pose_index'])
            run_files = glob.glob(os.path.join(csv_dir, f"*p{pose_idx:04d}*.csv"))
            if not run_files:
                continue
            try:
                run_df = pd.read_csv(run_files[0], skiprows=1, names=COLUMN_NAMES)
                run_df = pd.to_numeric(run_df.stack(), errors='coerce').unstack()
                mid_run_end_x.append(run_df['actual_real_x'].values[-1])
                mid_run_end_y.append(run_df['actual_real_y'].values[-1])
            except Exception:
                continue

        if mid_run_end_x:
            ax.scatter(
                mid_run_end_x, mid_run_end_y,
                c='red', s=60, marker='X', zorder=5,
                label=f'Mid-run collision (n={len(mid_run_end_x)})'
            )

        ax.set_title('All Collision Positions on Map', fontsize=13, fontweight='bold')
        ax.legend(loc='upper right')
        plt.tight_layout()
        out_all = os.path.join(output_dir, 'collision_all_positions.pdf')
        plt.savefig(out_all, dpi=150, bbox_inches='tight')
        print(f"Saved: {out_all}")
        
        plt.close()


# ─────────────────────────────────────────────
# ANALYSIS 3 — SPATIAL HEATMAP OF BIMODAL EVENTS
# ─────────────────────────────────────────────

def plot_bimodal_heatmap(df, map_img):
    """
    Heatmap of where bimodal belief occurs across all runs.
    Uses actual robot position (actual_real_x/y) at steps where
    is_bimodal = 1.0
    """
    bimodal_steps = df[df['is_bimodal'] == 1.0]    
    all_steps     = df

    x_range, y_range = get_ranges(map_img, df)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle('Spatial Distribution of Bimodal Belief',
                 fontsize=14, fontweight='bold')

    # Left: all visited positions as reference
    ax = axes[0]
    setup_axis(ax, map_img)
    h = build_heatmap(all_steps['actual_real_x'], all_steps['actual_real_y'],
                      x_range, y_range)
    draw_heatmap(ax, h, x_range, y_range, 'Visit density')
    ax.set_title(f'All Visited Positions\n({len(all_steps):,} steps)')

    # Right: bimodal steps only
    ax = axes[1]
    setup_axis(ax, map_img)
    if len(bimodal_steps) == 0:
        ax.set_title('No bimodal steps found\n(check is_bimodal)')
    else:
        h = build_heatmap(bimodal_steps['actual_real_x'], bimodal_steps['actual_real_y'],
                          x_range, y_range)
        draw_heatmap(ax, h, x_range, y_range, 'Bimodal event density')
        pct = 100 * len(bimodal_steps) / len(all_steps)
        ax.set_title(f'Bimodal Belief Locations\n'
                     f'({len(bimodal_steps):,} steps, {pct:.1f}% of all steps)')

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'bimodal_heatmap.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()


# ─────────────────────────────────────────────
# ANALYSIS 4 — BIMODALITY
# ─────────────────────────────────────────────

def plot_bimodal_action_preference(df):
    """
    Compare action distribution during bimodal vs non_bimodal belief.
    Shows whether the algorithm prefers orientation-gathering actions
    (ROTATE_LEFT/RIGHT) when belief is ambiguous.
    """
    bimodal   = df[df['is_bimodal'] == 1.0]
    non_bimodal  = df[df['is_bimodal'] == 0.0]

    action_order = list(ACTION_MAP.values())  # consistent ordering

    def get_pct(data):
        counts = data['action_name'].value_counts()
        pct    = counts.reindex(action_order, fill_value=0)
        return 100 * pct / pct.sum() if pct.sum() > 0 else pct

    bimodal_pct  = get_pct(bimodal)
    non_bimodal_pct = get_pct(non_bimodal)

    x      = np.arange(len(action_order))
    width  = 0.38

    fig, ax = plt.subplots(figsize=(12, 5))
    bars1 = ax.bar(x - width/2, non_bimodal_pct.values, width,
                   label=f'non_bimodal (n={len(non_bimodal):,})',
                   color='steelblue', alpha=0.85)
    bars2 = ax.bar(x + width/2, bimodal_pct.values,  width,
                   label=f'Bimodal  (n={len(bimodal):,})',
                   color='tomato',    alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels(action_order, rotation=30, ha='right')
    ax.set_ylabel('Action frequency (%)')
    ax.set_title('Action Preference: Bimodal vs non_bimodal Belief',
                 fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    # Annotate difference
    for i, (b, u) in enumerate(zip(bimodal_pct.values, non_bimodal_pct.values)):
        diff = b - u
        if abs(diff) > 1.0:
            ax.text(i + width/2, b + 0.3, f'{diff:+.1f}%',
                    ha='center', va='bottom', fontsize=7,
                    color='darkred' if diff > 0 else 'darkblue')

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'bimodal_action_preference.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

def plot_bimodal_decision_analysis(df, map_img):

    bimodal = df[df['is_bimodal'] == 1.0].copy()
    if len(bimodal) == 0:
        print("  No bimodal steps found.")
        return

    # ── For each run, find bimodal episodes and their resolution ─
    # A bimodal episode ends when is_bimodal drops to 0.
    # At resolution, check which peak the belief (weighted mean cluster)
    # converged on, and whether that matches GT.

    episode_results = []  # one entry per bimodal episode per run

    for run_id, run_df in df.groupby('run_id'):
        run_df = run_df.sort_values('step').reset_index(drop=True)

        # Find transitions: bimodal → not bimodal
        was_bimodal  = run_df['is_bimodal'] == 1.0
        transitions  = was_bimodal.astype(int).diff()
        episode_ends = run_df[transitions == -1].index  # rows where bimodal just ended

        if len(episode_ends) == 0:
            # Run ended while still bimodal — use last step as resolution
            if was_bimodal.any():
                episode_ends = [run_df.index[-1]]
            else:
                continue

        for end_idx in episode_ends:
            # Steps in this episode = all preceding consecutive bimodal steps
            episode_rows = run_df.loc[:end_idx][run_df.loc[:end_idx, 'is_bimodal'] == 1.0]
            if len(episode_rows) == 0:
                continue

            # Resolution step = first non-bimodal step after episode
            if end_idx < run_df.index[-1]:
                resolution_row = run_df.loc[end_idx]
            else:
                resolution_row = run_df.iloc[-1]

            # ── Which peak did belief converge on? ───────────────
            # Use weighted mean cluster position as proxy for final belief
            belief_x = resolution_row['x_weighted_mean_cluster']
            belief_y = resolution_row['y_weighted_mean_cluster']

            # Last bimodal step — peak positions
            last_bimodal = episode_rows.iloc[-1]
            dist_belief_to_p1 = np.sqrt((belief_x - last_bimodal['peak1_x'])**2 +
                                         (belief_y - last_bimodal['peak1_y'])**2)
            dist_belief_to_p2 = np.sqrt((belief_x - last_bimodal['peak2_x'])**2 +
                                         (belief_y - last_bimodal['peak2_y'])**2)

            converged_on = 'peak1' if dist_belief_to_p1 < dist_belief_to_p2 else 'peak2'

            # ── Which peak was the correct one? ──────────────────
            # Correct = peak closer to GT position
            gt_x = last_bimodal['actual_real_x']
            gt_y = last_bimodal['actual_real_y']
            dist_gt_to_p1 = np.sqrt((gt_x - last_bimodal['peak1_x'])**2 +
                                     (gt_y - last_bimodal['peak1_y'])**2)
            dist_gt_to_p2 = np.sqrt((gt_x - last_bimodal['peak2_x'])**2 +
                                     (gt_y - last_bimodal['peak2_y'])**2)
            correct_peak = 'peak1' if dist_gt_to_p1 < dist_gt_to_p2 else 'peak2'

            resolved_correctly = (converged_on == correct_peak)

            episode_results.append({
                'run_id':              run_id,
                'episode_length':      len(episode_rows),
                'episode_start_step':  episode_rows.iloc[0]['step'],
                'gt_x':                gt_x,
                'gt_y':                gt_y,
                'correct_peak':        correct_peak,
                'converged_on':        converged_on,
                'resolved_correctly':  resolved_correctly,
                'actions':             episode_rows['action_name'].tolist(),
            })

    if not episode_results:
        print("  No bimodal episodes with resolution found.")
        return

    episodes_df = pd.DataFrame(episode_results)
    correct_ep   = episodes_df[episodes_df['resolved_correctly'] == True]
    incorrect_ep = episodes_df[episodes_df['resolved_correctly'] == False]

    print(f"  Bimodal episodes total     : {len(episodes_df)}")
    print(f"  Resolved to correct peak   : {len(correct_ep)} "
          f"({100*len(correct_ep)/len(episodes_df):.1f}%)")
    print(f"  Resolved to wrong peak     : {len(incorrect_ep)} "
          f"({100*len(incorrect_ep)/len(episodes_df):.1f}%)")

    # ── Flatten actions per episode for action analysis ───────────
    def get_action_pct(ep_subset):
        all_actions = [a for actions in ep_subset['actions'] for a in actions]
        s = pd.Series(all_actions).value_counts()
        s = s.reindex(list(ACTION_MAP.values()), fill_value=0)
        return 100 * s / s.sum() if s.sum() > 0 else s

    correct_pct   = get_action_pct(correct_ep)
    incorrect_pct = get_action_pct(incorrect_ep)

    # ── Action type helper ────────────────────────────────────────
    def action_type(action_name):
        p = ACTION_PARAMS.get(action_name, {'linear': 0.0, 'angular': 0.0})
        has_t = abs(p['linear'])  > 1e-6
        has_r = abs(p['angular']) > 1e-6
        if has_t and has_r: return 'mixed'
        elif has_t:         return 'translation'
        elif has_r:         return 'rotation'
        else:               return 'wait'

    # ── FIGURE ───────────────────────────────────────────────────
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('Bimodal Belief Resolution — Correct vs Wrong Peak Convergence',
                 fontsize=14, fontweight='bold')

    # ── Plot 1: Map — where did correct/incorrect resolutions occur
    ax1 = fig.add_subplot(2, 3, (1, 2))
    setup_axis(ax1, map_img)
    if len(correct_ep) > 0:
        ax1.scatter(correct_ep['gt_x'], correct_ep['gt_y'],
                    c='seagreen', s=30, alpha=0.7, zorder=4,
                    label=f'Resolved correctly (n={len(correct_ep)})')
    if len(incorrect_ep) > 0:
        ax1.scatter(incorrect_ep['gt_x'], incorrect_ep['gt_y'],
                    c='tomato', s=30, alpha=0.7, zorder=4,
                    label=f'Resolved to wrong peak (n={len(incorrect_ep)})')
    ax1.set_title(f'Where Did Bimodal Belief Resolve?\n'
                  f'({len(episodes_df)} episodes across {episodes_df["run_id"].nunique()} runs)')
    ax1.legend(loc='upper right', markerscale=1.5, fontsize=9)

    # ── Plot 2: Episode length distribution ──────────────────────
    ax2 = fig.add_subplot(2, 3, 3)
    bins = range(1, int(episodes_df['episode_length'].max()) + 2)
    ax2.hist(correct_ep['episode_length'],   bins=bins, alpha=0.7,
             color='seagreen', label=f'Correct (mean={correct_ep["episode_length"].mean():.1f})')
    ax2.hist(incorrect_ep['episode_length'], bins=bins, alpha=0.7,
             color='tomato',   label=f'Wrong   (mean={incorrect_ep["episode_length"].mean():.1f})')
    ax2.set_xlabel('Episode length (steps)')
    ax2.set_ylabel('Count')
    ax2.set_title('Bimodal Episode Duration\n(does longer = more likely correct?)')
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)

    # ── Plot 3: Action distribution — correct vs wrong resolution ─
    ax3 = fig.add_subplot(2, 3, (4, 5))
    action_order = list(ACTION_MAP.values())
    x     = np.arange(len(action_order))
    width = 0.38
    ax3.bar(x - width/2, correct_pct.values,   width,
            label=f'Episodes → correct peak (n={len(correct_ep)})',
            color='seagreen', alpha=0.85)
    ax3.bar(x + width/2, incorrect_pct.values, width,
            label=f'Episodes → wrong peak (n={len(incorrect_ep)})',
            color='tomato', alpha=0.85)

    # Annotate meaningful differences
    for i, (c, w) in enumerate(zip(correct_pct.values, incorrect_pct.values)):
        diff = c - w
        if abs(diff) > 2.0:
            ax3.text(i - width/2, c + 0.3, f'{diff:+.1f}%',
                     ha='center', va='bottom', fontsize=7,
                     color='darkgreen' if diff > 0 else 'darkred')

    ax3.set_xticks(x)
    ax3.set_xticklabels(action_order, rotation=30, ha='right')
    ax3.set_ylabel('Action frequency during episode (%)')
    ax3.set_title('Which Actions During Bimodal Episode\nLead to Correct Belief Resolution?')
    ax3.legend()
    ax3.grid(axis='y', alpha=0.3)

    # ── Plot 4: Correct resolution rate by dominant action type ──
    ax4 = fig.add_subplot(2, 3, 6)

    type_results = []
    for atype in ['translation', 'rotation', 'mixed', 'wait']:
        # Dominant action type = most frequent type in episode
        def dominant_type(actions):
            types = [action_type(a) for a in actions]
            return pd.Series(types).value_counts().index[0]

        episodes_df['dominant_type'] = episodes_df['actions'].apply(dominant_type)
        sub = episodes_df[episodes_df['dominant_type'] == atype]
        if len(sub) == 0:
            continue
        rate = 100 * sub['resolved_correctly'].sum() / len(sub)
        type_results.append((atype, rate, len(sub)))

    if type_results:
        type_colors = {'translation': 'steelblue', 'rotation': 'tomato',
                       'mixed': 'seagreen', 'wait': 'gray'}
        labels = [f"{r[0]}\n(n={r[2]})" for r in type_results]
        values = [r[1] for r in type_results]
        colors = [type_colors[r[0]] for r in type_results]
        ax4.bar(labels, values, color=colors, alpha=0.85)
        ax4.axhline(50, color='black', linestyle='--', linewidth=1, label='50% chance')
        ax4.axhline(100 * len(correct_ep) / max(len(episodes_df), 1),
                    color='gray', linestyle=':', linewidth=1, label='Overall rate')
        ax4.set_ylabel('% episodes resolved to correct peak')
        ax4.set_ylim(0, 100)
        ax4.set_title('Correct Resolution Rate\nby Dominant Action Type')
        ax4.legend(fontsize=8)
        ax4.grid(axis='y', alpha=0.3)

    # ── Overall stat ──────────────────────────────────────────────
    overall = 100 * len(correct_ep) / max(len(episodes_df), 1)
    fig.text(0.5, 0.01,
             f'Overall correct belief resolution: {overall:.1f}%  |  '
             f'Mean episode length: {episodes_df["episode_length"].mean():.1f} steps  |  '
             f'Mean episode time: {episodes_df["episode_length"].mean() * ACTION_DT:.1f}s',
             ha='center', fontsize=11, style='italic')

    plt.tight_layout(rect=[0, 0.03, 1, 1])
    out = os.path.join(OUTPUT_DIR, 'bimodal_decision_analysis.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()
# ─────────────────────────────────────────────
# ANALYSIS 5 — RUN STATISTICS REPORT (txt)
# ─────────────────────────────────────────────

def save_run_statistics(df, csv_dir, output_dir):
    """
    Per-run statistics txt report. Excludes step-0 collision runs entirely.
    Covers:
      - Steps per run (min, max, mean)
      - Bimodal resolution time (steps spent in bimodal belief per run)
      - Success breakdown:
          * True success:    converged + low pos & rot error
          * False success:   converged + high pos or rot error
          * Collision:       mid-run collision (steps > 0)
          * Other failure:   timeout, unknown
    """
    valid_summary = load_valid_summary(csv_dir)

    # ── Per-run step counts from df ──────────────────────────────────────
    steps_per_run = df.groupby('run_id')['step'].count()

    bimodal_per_run = (df[df['is_bimodal'] == 1.0]
                       .groupby('run_id')['step'].count())

    # ── Final step error per run (last row = state at convergence/end) ───
    final_per_run = df.sort_values('step').groupby('run_id').last()

    # ── Classify each run ────────────────────────────────────────────────
    def classify(row):
        status = str(row.get('status', ''))
        steps  = row.get('steps', 0)

        if 'Collision' in status and steps > 0:
            return 'mid_run_collision'

        if 'Convergence' in status or 'SUCCESS' in status:
            run_id   = row.get('pose_index', -1) - 1  # pose_index is 1-based
            # Look up final errors from the step data
            if run_id in final_per_run.index:
                pos_err = final_per_run.loc[run_id, 'position_error']
                rot_err = final_per_run.loc[run_id, 'rotational_error']
                if pd.notna(pos_err) and pd.notna(rot_err):
                    if pos_err <= POS_ERROR_THRESHOLD and rot_err <= ROT_ERROR_THRESHOLD:
                        return 'true_success'
                    else:
                        return 'false_success'
            return 'true_success'  # no error data available, assume ok

        if 'TIMEOUT' in status or 'Timeout' in status:
            return 'timeout'

        return 'other_failure'

    valid_summary['outcome'] = valid_summary.apply(classify, axis=1)

    # ── Counts ────────────────────────────────────────────────────────────
    total           = len(valid_summary)
    true_success    = (valid_summary['outcome'] == 'true_success').sum()
    false_success   = (valid_summary['outcome'] == 'false_success').sum()
    mid_collision   = (valid_summary['outcome'] == 'mid_run_collision').sum()
    timeout         = (valid_summary['outcome'] == 'timeout').sum()
    other           = (valid_summary['outcome'] == 'other_failure').sum()

    def pct(n): return 100 * n / total if total > 0 else 0.0

    # ── Write report ─────────────────────────────────────────────────────
    out = os.path.join(output_dir, 'run_statistics.txt')
    with open(out, 'w') as f:

        f.write('='*65 + '\n')
        f.write('RUN STATISTICS REPORT\n')
        f.write(f'Thresholds: pos_error <= {POS_ERROR_THRESHOLD}m, '
                f'rot_error <= {ROT_ERROR_THRESHOLD} rad\n')
        f.write(f'Bimodal threshold: is_bimodal\n')
        f.write('(Step-0 collision runs excluded from all statistics)\n')
        f.write('='*65 + '\n\n')

        # ── Step statistics ──────────────────────────────────────────────
        f.write('STEPS PER RUN\n')
        f.write('-'*40 + '\n')
        f.write(f'  Total valid runs : {total}\n')
        f.write(f'  Min steps        : {int(steps_per_run.min())}\n')
        f.write(f'  Max steps        : {int(steps_per_run.max())}\n')
        f.write(f'  Mean steps       : {steps_per_run.mean():.2f}\n')
        f.write(f'  Median steps     : {steps_per_run.median():.1f}\n')
        f.write(f'  Std steps        : {steps_per_run.std():.2f}\n\n')

        # ── Bimodal resolution ───────────────────────────────────────────
        f.write('BIMODAL BELIEF DURATION\n')
        f.write('-'*40 + '\n')
        runs_with_bimodal = bimodal_per_run[bimodal_per_run > 0]
        if len(runs_with_bimodal) == 0:
            f.write('  No bimodal episodes detected.\n\n')
        else:
            f.write(f'  Runs with bimodal belief  : {len(runs_with_bimodal)} '
                    f'({100*len(runs_with_bimodal)/total:.1f}% of valid runs)\n')
            f.write(f'  Min bimodal steps/run     : {int(runs_with_bimodal.min())}\n')
            f.write(f'  Max bimodal steps/run     : {int(runs_with_bimodal.max())}\n')
            f.write(f'  Mean bimodal steps/run    : {runs_with_bimodal.mean():.2f}\n')
            f.write(f'  Mean bimodal time/run     : '
                    f'{runs_with_bimodal.mean() * ACTION_DT:.2f}s\n')
            f.write(f'  Median bimodal steps/run  : {runs_with_bimodal.median():.1f}\n\n')

        # ── Outcome breakdown ────────────────────────────────────────────
        f.write('OUTCOME BREAKDOWN\n')
        f.write('-'*40 + '\n')
        f.write(f'  True success          : {true_success:>4}  ({pct(true_success):.1f}%)\n')
        f.write(f'    pos_err <= {POS_ERROR_THRESHOLD}m AND rot_err <= {ROT_ERROR_THRESHOLD} rad\n')
        f.write(f'  False success         : {false_success:>4}  ({pct(false_success):.1f}%)\n')
        f.write(f'    converged but high pos or rot error\n')
        f.write(f'  Mid-run collision     : {mid_collision:>4}  ({pct(mid_collision):.1f}%)\n')
        f.write(f'  Timeout               : {timeout:>4}  ({pct(timeout):.1f}%)\n')
        f.write(f'  Other failure         : {other:>4}  ({pct(other):.1f}%)\n')
        f.write(f'  ─────────────────────────────\n')
        f.write(f'  Total                 : {total:>4}\n\n')

        # ── Error distribution for successful runs ───────────────────────
        success_runs = final_per_run[
            final_per_run.index.isin(
                valid_summary[valid_summary['outcome'].isin(
                    ['true_success', 'false_success'])].index
            )
        ]
        if len(success_runs) > 0:
            f.write('FINAL ERROR DISTRIBUTION (converged runs)\n')
            f.write('-'*40 + '\n')
            f.write(f'  Position error:\n')
            f.write(f'    Mean   : {success_runs["position_error"].mean():.4f} m\n')
            f.write(f'    Median : {success_runs["position_error"].median():.4f} m\n')
            f.write(f'    Std    : {success_runs["position_error"].std():.4f} m\n')
            f.write(f'    Min    : {success_runs["position_error"].min():.4f} m\n')
            f.write(f'    Max    : {success_runs["position_error"].max():.4f} m\n\n')
            f.write(f'  Rotational error:\n')
            f.write(f'    Mean   : {success_runs["rotational_error"].mean():.4f} rad\n')
            f.write(f'    Median : {success_runs["rotational_error"].median():.4f} rad\n')
            f.write(f'    Std    : {success_runs["rotational_error"].std():.4f} rad\n')
            f.write(f'    Min    : {success_runs["rotational_error"].min():.4f} rad\n')
            f.write(f'    Max    : {success_runs["rotational_error"].max():.4f} rad\n\n')

        # ── Rotational error distribution analysis ───────────────────
        f.write('ROTATIONAL ERROR DEEP DIVE\n')
        f.write('-'*40 + '\n')

        all_final = df.sort_values('step').groupby('run_id').last()

        rot_errors = all_final['rotational_error'].dropna()

        # Bin into brackets to see distribution shape
        brackets = [
            (0.0,  0.1,  'Excellent  (< 0.1 rad  / ~6°)'),
            (0.1,  0.3,  'Good       (0.1–0.3 rad / ~6–17°)'),
            (0.3,  0.5,  'Acceptable (0.3–0.5 rad / ~17–29°)'),
            (0.5,  1.0,  'Poor       (0.5–1.0 rad / ~29–57°)'),
            (1.0,  1.6,  'Bad        (1.0–1.6 rad / ~57–92°)'),
            (1.6,  2.4,  'Very bad   (1.6–2.4 rad / ~92–137°)'),
            (2.4,  3.15, 'Near flip  (2.4–3.15 rad / ~137–180°)'),
        ]
        total_rot = len(rot_errors)
        f.write(f'  Total runs with final rot error : {total_rot}\n\n')
        for lo, hi, label in brackets:
            n   = ((rot_errors >= lo) & (rot_errors < hi)).sum()
            bar = '█' * int(30 * n / total_rot) if total_rot > 0 else ''
            f.write(f'  {label:<40} : {n:>4} ({100*n/total_rot:5.1f}%)  {bar}\n')

        # Check for ~pi clustering — strong sign of 180° flip / symmetric confusion
        near_pi = ((rot_errors >= 2.8) & (rot_errors <= 3.15)).sum()
        near_pi_pct = 100 * near_pi / total_rot if total_rot > 0 else 0
        f.write(f'\n  Near-180° flips (2.8–3.15 rad) : {near_pi} ({near_pi_pct:.1f}%)\n')
        if near_pi_pct > 10:
            f.write(f'  *** HIGH: algorithm is converging on symmetric/flipped hypothesis\n')

        # Check for ~pi/2 clustering — sign of 90° map confusion
        near_half_pi = ((rot_errors >= 1.4) & (rot_errors <= 1.7)).sum()
        near_half_pi_pct = 100 * near_half_pi / total_rot if total_rot > 0 else 0
        f.write(f'  Near-90° errors  (1.4–1.7 rad) : {near_half_pi} ({near_half_pi_pct:.1f}%)\n')
        if near_half_pi_pct > 10:
            f.write(f'  *** HIGH: algorithm is confusing perpendicular corridors\n')

        # Distribution over all steps — not just final — to see if it improves over time
        f.write(f'\n  Rotational error over all steps (not just final):\n')
        all_rot = df['rotational_error'].dropna()
        f.write(f'    Mean   : {all_rot.mean():.4f} rad\n')
        f.write(f'    Median : {all_rot.median():.4f} rad\n')
        f.write(f'    Std    : {all_rot.std():.4f} rad\n')
        f.write(f'    > 1.0 rad : {(all_rot > 1.0).sum():>5} steps '
                f'({100*(all_rot > 1.0).mean():.1f}% of all steps)\n')
        f.write(f'    > 2.0 rad : {(all_rot > 2.0).sum():>5} steps '
                f'({100*(all_rot > 2.0).mean():.1f}% of all steps)\n\n')

        # Compare true success vs false success rot error
        true_ids  = valid_summary[valid_summary['outcome'] == 'true_success']['pose_index'] - 1
        false_ids = valid_summary[valid_summary['outcome'] == 'false_success']['pose_index'] - 1

        true_rot  = all_final.loc[all_final.index.isin(true_ids),  'rotational_error'].dropna()
        false_rot = all_final.loc[all_final.index.isin(false_ids), 'rotational_error'].dropna()

        f.write(f'  Rot error — true success runs:\n')
        f.write(f'    Mean: {true_rot.mean():.4f} rad  |  '
                f'Median: {true_rot.median():.4f} rad\n')
        f.write(f'  Rot error — false success runs:\n')
        f.write(f'    Mean: {false_rot.mean():.4f} rad  |  '
                f'Median: {false_rot.median():.4f} rad\n\n')
        
        
        # ── False success detail ─────────────────────────────────────────
        if false_success > 0:
            false_runs = valid_summary[valid_summary['outcome'] == 'false_success']
            f.write('FALSE SUCCESS DETAILS\n')
            f.write('-'*40 + '\n')
            for _, row in false_runs.iterrows():
                rid = int(row['pose_index']) - 1
                if rid in final_per_run.index:
                    pe = final_per_run.loc[rid, 'position_error']
                    re = final_per_run.loc[rid, 'rotational_error']
                    f.write(f'  pose {int(row["pose_index"]):04d} | '
                            f'pos_err={pe:.3f}m | rot_err={re:.3f}rad\n')
            f.write('\n')

    print(f"Saved: {out}")

# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────

def main():
    print("Loading CSVs...")
    df = load_all_csvs(CSV_DIR)
    print("Loading map...")
    map_img = load_map(MAP_PATH)
    if map_img is not None:
        h, w = map_img.shape[:2]
        print(f"Map image size: {w} x {h} pixels")
        print(f"Map extent: x=[{MAP_ORIGIN_X:.3f}, {MAP_ORIGIN_X + w * MAP_RESOLUTION:.3f}]")
        print(f"            y=[{MAP_ORIGIN_Y:.3f}, {MAP_ORIGIN_Y + h * MAP_RESOLUTION:.3f}]")
    
    print("Preparing dataframe...")
    df = prepare_df(df)

    print(f"\nDataset summary:")
    print(f"  Total steps      : {len(df):,}")
    print(f"  Total runs       : {df['run_id'].nunique()}")
    print(f"  Action counts:\n{df['action_name'].value_counts().to_string()}")

    print("\nPlotting starting positions...")
    plot_starting_positions(df, map_img)
    
    print("\nPlotting position heatmap...")
    plot_position_heatmap(df, map_img)

    print("\nPlotting gaze heatmap...")
    plot_gaze_heatmap(df, map_img)

    print("\nPlotting quiver overlay...")
    plot_quiver_overlay(df, map_img)

    print("\nSaving collision report...")
    save_collision_report(CSV_DIR, OUTPUT_DIR)

    print("\nPlotting bimodal heatmap...")
    plot_bimodal_heatmap(df, map_img)

    print("\nPlotting bimodal action preference...")
    plot_bimodal_action_preference(df)

    print("\nSaving run statistics...")
    save_run_statistics(df, CSV_DIR, OUTPUT_DIR)

    print("\nPlotting bimodal decision_analysis...")
    plot_bimodal_decision_analysis(df, map_img)

    print(f"\nDone. Figures saved to: {OUTPUT_DIR}")

if __name__ == '__main__':
    main()