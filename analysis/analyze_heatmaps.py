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

CSV_DIR = os.path.join(PARENT_ROOT, "results", "0603-numba-5") # ../results/ — outside repo
OUTPUT_DIR = os.path.join(PARENT_ROOT, "figures")  # ../figures/ — outside repo
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
    # Starting position = first step of each run
    starts = df[df['step'] == 0][['actual_real_x', 'actual_real_y', 'run_id']].copy()

    fig, ax = plt.subplots(figsize=(10, 9))
    setup_axis(ax, map_img)

    sc = ax.scatter(
        starts['actual_real_x'], starts['actual_real_y'],
        c='red', s=20, alpha=0.6, zorder=3, label=f'Start positions (n={len(starts)})'
    )
    ax.set_title(f'Starting Positions Across All Runs\n({len(starts)} runs)',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper right')

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, 'starting_positions.pdf')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.show()

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
    plt.show()

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
    plt.show()

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
    plt.show()

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
            plt.show()
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

    print(f"\nDone. Figures saved to: {OUTPUT_DIR}")

if __name__ == '__main__':
    main()