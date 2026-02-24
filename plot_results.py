#!/usr/bin/env python3
"""
Plot ground truth (MotionLog) vs algorithm results (AlgoLog) over time.
Displays position (x, y, z) and rotation angles (pitch, yaw, roll).
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_motion_log(filepath):
    """Load MotionLog CSV (ground truth)."""
    df = pd.read_csv(filepath)
    # Rename columns for consistency
    df.rename(columns={
        'Time': 'time',
        'PosX': 'pos_x',
        'PosY': 'pos_y',
        'PosZ': 'pos_z',
        'Pitch': 'pitch',
        'Yaw': 'yaw',
        'Roll': 'roll',
        'PitchCam': 'rel_pitch',
        'YawCam': 'rel_yaw',
        'RollCam': 'rel_roll'
    }, inplace=True)
    return df

def load_algo_log(filepath):
    """Load AlgoLog CSV (algorithm results)."""
    df = pd.read_csv(filepath)
    # Rename columns for consistency
    df.rename(columns={
        'img_ts': 'time',
        'tx_px': 'pos_x',
        'ty_px': 'pos_y',
        'rvec_z': 'rotation_z',  # Approximate mapping
        'rvec_x': 'rotation_x',
        'rvec_y': 'rotation_y'
    }, inplace=True)
    # Note: AlgoLog doesn't have z position, so we'll skip it for algo
    return df

def plot_comparison(motion_log_1, motion_log_2, output_file='comparison_plot_poses.png'):
    """Create comparison plots between MotionLog 1 (ground truth) and MotionLog 2 (UE path).
    
    Normalizes time to start at 0 (referenced from motion_log_2's first timestamp).
    Uses motion_log_2's time axis for both datasets (motion_log_1 values are constant, so time doesn't matter).
    Extends constant ground truth values across the entire time duration.
    """
    fig, axes = plt.subplots(9, 1, figsize=(21, 16))
    fig.suptitle('Ground Truth (MotionLog 1) vs Unreal Engine Path (MotionLog 2)', fontsize=16, fontweight='bold')
    
    # Normalize time: start from 0 (reference from motion_log_2's first time)
    time_offset = motion_log_2['time'].iloc[0]
    motion_log_2_time_normalized = motion_log_2['time'] - time_offset
    
    # Get constant values from motion_log_1 (use first value since they're all constant)
    const_pos_x = motion_log_1['pos_x'].iloc[0]
    const_pos_y = motion_log_1['pos_y'].iloc[0]
    const_pos_z = motion_log_1['pos_z'].iloc[0]
    const_pitch = motion_log_1['pitch'].iloc[0]
    const_yaw = motion_log_1['yaw'].iloc[0]
    const_roll = motion_log_1['roll'].iloc[0]
    const_rel_pitch = motion_log_1['rel_pitch'].iloc[0]
    const_rel_yaw = motion_log_1['rel_yaw'].iloc[0]
    const_rel_roll = motion_log_1['rel_roll'].iloc[0]
    
    # Create constant arrays with same length as motion_log_2
    n_points = len(motion_log_2)
    gt_pos_x = np.full(n_points, const_pos_x)
    gt_pos_y = np.full(n_points, const_pos_y)
    gt_pos_z = np.full(n_points, const_pos_z)
    gt_pitch = np.full(n_points, const_pitch)
    gt_yaw = np.full(n_points, const_yaw)
    gt_roll = np.full(n_points, const_roll)
    gt_rel_pitch = np.full(n_points, const_rel_pitch)
    gt_rel_yaw = np.full(n_points, const_rel_yaw)
    gt_rel_roll = np.full(n_points, const_rel_roll)
    
    # Plot Position X
    axes[0].plot(motion_log_2_time_normalized, gt_pos_x, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[0].plot(motion_log_2_time_normalized, motion_log_2['pos_x'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[0].set_ylabel('Position X (pixels)', fontsize=10)
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Position Y
    axes[1].plot(motion_log_2_time_normalized, gt_pos_y, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[1].plot(motion_log_2_time_normalized, motion_log_2['pos_y'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[1].set_ylabel('Position Y (pixels)', fontsize=10)
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)
    
    # Plot Position Z
    axes[2].plot(motion_log_2_time_normalized, gt_pos_z, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[2].plot(motion_log_2_time_normalized, motion_log_2['pos_z'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[2].set_ylabel('Position Z (pixels)', fontsize=10)
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)
    
    # Plot Rotation X (Pitch)
    axes[3].plot(motion_log_2_time_normalized, gt_pitch, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[3].plot(motion_log_2_time_normalized, motion_log_2['pitch'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[3].set_ylabel('Pitch (degrees)', fontsize=10)
    axes[3].legend(loc='best')
    axes[3].grid(True, alpha=0.3)
    
    # Plot Rotation Y (Yaw)
    axes[4].plot(motion_log_2_time_normalized, gt_yaw, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[4].plot(motion_log_2_time_normalized, motion_log_2['yaw'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[4].set_ylabel('Yaw (degrees)', fontsize=10)
    axes[4].legend(loc='best')
    axes[4].grid(True, alpha=0.3)
    
    # Plot Rotation Z (Roll)
    axes[5].plot(motion_log_2_time_normalized, gt_roll, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[5].plot(motion_log_2_time_normalized, motion_log_2['roll'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[5].set_ylabel('Roll (degrees)', fontsize=10)
    axes[5].legend(loc='best')
    axes[5].grid(True, alpha=0.3)
    
    # Plot Relative Rotation X (Relative Pitch)
    axes[6].plot(motion_log_2_time_normalized, gt_rel_pitch, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[6].plot(motion_log_2_time_normalized, motion_log_2['rel_pitch'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[6].set_ylabel('Rel. Pitch (degrees)', fontsize=10)
    axes[6].legend(loc='best')
    axes[6].grid(True, alpha=0.3)
    
    # Plot Relative Rotation Y (Relative Yaw)
    axes[7].plot(motion_log_2_time_normalized, gt_rel_yaw, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[7].plot(motion_log_2_time_normalized, motion_log_2['rel_yaw'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[7].set_ylabel('Rel. Yaw (degrees)', fontsize=10)
    axes[7].legend(loc='best')
    axes[7].grid(True, alpha=0.3)
    
    # Plot Relative Rotation Z (Relative Roll)
    axes[8].plot(motion_log_2_time_normalized, gt_rel_roll, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[8].plot(motion_log_2_time_normalized, motion_log_2['rel_roll'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[8].set_ylabel('Rel. Roll (degrees)', fontsize=10)
    axes[8].set_xlabel('Time (seconds)', fontsize=10)
    axes[8].legend(loc='best')
    axes[8].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Poses comparison plot saved to {output_file}")
    plt.show()

def plot_algo_log(algo_log, output_file='algo_log_plot.png'):
    """Create plot showing AlgoLog results."""
    fig, axes = plt.subplots(6, 1, figsize=(14, 12))
    fig.suptitle('Algorithm Results (AlgoLog)', fontsize=16, fontweight='bold')
    
    # Plot Position X
    axes[0].plot(algo_log['time'], algo_log['pos_x'], 'g-', linewidth=2)
    axes[0].set_ylabel('Position X (pixels)', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    
    # Plot Position Y
    axes[1].plot(algo_log['time'], algo_log['pos_y'], 'g-', linewidth=2)
    axes[1].set_ylabel('Position Y (pixels)', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    
    # Plot Rotation X
    axes[2].plot(algo_log['time'], algo_log['rotation_x'], 'g-', linewidth=2)
    axes[2].set_ylabel('Rotation X (rad)', fontsize=10)
    axes[2].grid(True, alpha=0.3)
    
    # Plot Rotation Y
    axes[3].plot(algo_log['time'], algo_log['rotation_y'], 'g-', linewidth=2)
    axes[3].set_ylabel('Rotation Y (rad)', fontsize=10)
    axes[3].grid(True, alpha=0.3)
    
    # Plot Rotation Z
    axes[4].plot(algo_log['time'], algo_log['rotation_z'], 'g-', linewidth=2)
    axes[4].set_ylabel('Rotation Z (rad)', fontsize=10)
    axes[4].grid(True, alpha=0.3)
    
    # Plot Commands combined (cmd0, cmd1, cmd2)
    axes[5].plot(algo_log['time'], algo_log['cmd0'], label='cmd0', linewidth=2)
    axes[5].plot(algo_log['time'], algo_log['cmd1'], label='cmd1', linewidth=2)
    axes[5].plot(algo_log['time'], algo_log['cmd2'], label='cmd2', linewidth=2)
    axes[5].set_ylabel('Commands', fontsize=10)
    axes[5].set_xlabel('Time (seconds)', fontsize=10)
    axes[5].legend(loc='best')
    axes[5].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ AlgoLog plot saved to {output_file}")
    plt.show()

def main():
    # Find the latest log files
    repo_root = Path(__file__).parent
    
    # Get MotionLog files (ground truth)
    motion_logs = sorted(list(repo_root.glob('groundTruths/MotionLog_*.csv')))
    if not motion_logs:
        print("❌ No MotionLog files found in groundTruths/")
        return
    
    # Look for GT (ground truth) file
    gt_logs = sorted(list(repo_root.glob('groundTruths/*GT*.csv')))
    if gt_logs:
        motion_log_1_path = gt_logs[-1]  # Use the GT file
        # Get other motion logs for comparison
        other_logs = sorted([f for f in motion_logs if 'GT' not in f.name])
        motion_log_2_path = other_logs[-1] if other_logs else None
        print(f"Loading ground truth (poses): {motion_log_1_path.name}")
        if motion_log_2_path:
            print(f"Loading Unreal Engine path (rates): {motion_log_2_path.name}")
    else:
        # Fallback to old behavior if no GT file found
        if len(motion_logs) == 1:
            print(f"⚠️  Only one MotionLog file found: {motion_logs[0].name}")
            motion_log_1_path = motion_logs[0]
            motion_log_2_path = None
        else:
            motion_log_1_path = motion_logs[-2]  # Second-to-last
            motion_log_2_path = motion_logs[-1]  # Latest (Unreal Engine path)
            print(f"Loading ground truth (poses): {motion_log_1_path.name}")
            print(f"Loading Unreal Engine path (rates): {motion_log_2_path.name}")
    
    # Get AlgoLog (algorithm results)
    algo_logs = list(repo_root.glob('build/AlgoLog_*.csv'))
    if not algo_logs:
        print("❌ No AlgoLog files found in build/")
        return
    algo_log_path = algo_logs[-1]  # Get latest
    print(f"Loading algorithm results: {algo_log_path.name}")
    
    # Load data
    motion_df_1 = load_motion_log(motion_log_1_path)
    algo_df = load_algo_log(algo_log_path)
    
    print(f"\nMotionLog 1 (ground truth) shape: {motion_df_1.shape}")
    print(f"AlgoLog shape: {algo_df.shape}")
    
    # Create poses comparison plot (MotionLog 1 vs MotionLog 2)
    if motion_log_2_path:
        motion_df_2 = load_motion_log(motion_log_2_path)
        print(f"MotionLog 2 (UE path) shape: {motion_df_2.shape}")
        output_file_poses = repo_root / f'results/comparison_plot{motion_log_2_path.name[9:-4]}.png'
        plot_comparison(motion_df_1, motion_df_2, output_file_poses)
    else:
        print("⚠️  Skipping poses comparison (only one MotionLog file available)")
    
    # Create AlgoLog plot (standalone, no comparison)
    output_file_algo = repo_root / f'results/algo_log_plot{motion_log_2_path.name[9:-4]}.png'
    plot_algo_log(algo_df, output_file_algo)

if __name__ == '__main__':
    main()
