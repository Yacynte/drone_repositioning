#!/usr/bin/env python3
"""
Plot ground truth (MotionLog) vs algorithm results (AlgoLog) over time.
Displays position (x, y, z) and rotation angles (pitch, yaw, roll).
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from datetime import datetime
from skimage.metrics import structural_similarity as ssim
import cv2

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
        'v_x': 'vel_x',
        'v_y': 'vel_y',
        'v_z': 'vel_z',
        'w_x': 'omega_x',
        'w_y': 'omega_y',
        'w_z': 'omega_z'
    }, inplace=True)
    # Note: AlgoLog doesn't have z position, so we'll skip it for algo
    return df

def process_timestamps(val_list):
    # Process only if the item is a string, otherwise keep it as is
    return [
        datetime.fromisoformat(item).timestamp() 
        if isinstance(item, str) 
        else item 
        for item in val_list
    ]

def plot_comparison(motion_log_1, motion_log_2, output_file_rot='comparison_plot_rot.png', output_file_trans='comparison_plot_trans.png'):
    """Create comparison plots between MotionLog 1 (ground truth) and MotionLog 2 (UE path).
    
    Normalizes time to start at 0 (referenced from motion_log_2's first timestamp).
    Uses motion_log_2's time axis for both datasets (motion_log_1 values are constant, so time doesn't matter).
    Extends constant ground truth values across the entire time duration.
    """
    fig, axes = plt.subplots(2, 1, figsize=(21, 16))
    fig.suptitle('Ground Truth (MotionLog 1) vs Unreal Engine Path (MotionLog 2)', fontsize=16, fontweight='bold')

    fig1, axes1 = plt.subplots(3, 1, figsize=(21, 16))
    fig1.suptitle('Ground Truth (MotionLog 1) vs Unreal Engine Path (MotionLog 2)', fontsize=16, fontweight='bold')

    # Normalize time: start from 0 (reference from motion_log_2's first time)
    time_offset = motion_log_2['time'].iloc[0]
    time_array = np.array(process_timestamps(motion_log_2['time']))
    motion_log_2_time_normalized =  time_array - time_array[0]
    # motion_log_2_time_normalized = motion_log_2['time'] - time_offset
    
    # Get constant values from motion_log_1 (use first value since they're all constant)
    const_pos_x = motion_log_1['pos_x'].iloc[-1]
    const_pos_y = motion_log_1['pos_y'].iloc[-1]
    const_pos_z = motion_log_1['pos_z'].iloc[-1]
    const_pitch = motion_log_1['pitch'].iloc[-1]
    const_yaw = motion_log_1['yaw'].iloc[-1]
    const_roll = motion_log_1['roll'].iloc[-1]
    const_rel_pitch = motion_log_1['rel_pitch'].iloc[-1]
    const_rel_yaw = motion_log_1['rel_yaw'].iloc[-1]
    const_rel_roll = motion_log_1['rel_roll'].iloc[-1]
    
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
    axes1[0].plot(motion_log_2_time_normalized, gt_pos_x, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes1[0].plot(motion_log_2_time_normalized, motion_log_2['pos_x'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes1[0].set_ylabel('Position X (cm)', fontsize=10)
    axes1[0].legend(loc='best')
    axes1[0].grid(True, alpha=0.3)
    
    # Plot Position Y
    axes1[1].plot(motion_log_2_time_normalized, gt_pos_y, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes1[1].plot(motion_log_2_time_normalized, motion_log_2['pos_y'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes1[1].set_ylabel('Position Y (cm)', fontsize=10)
    axes1[1].legend(loc='best')
    axes1[1].grid(True, alpha=0.3)
    
    # Plot Position Z
    axes1[2].plot(motion_log_2_time_normalized, gt_pos_z, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes1[2].plot(motion_log_2_time_normalized, motion_log_2['pos_z'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes1[2].set_ylabel('Position Z (cm)', fontsize=10)
    axes1[2].legend(loc='best')
    axes1[2].grid(True, alpha=0.3)
    axes1[-1].set_xlabel('Time (s)', fontsize=12)
    fig1.tight_layout()
    fig1.savefig(output_file_trans, dpi=150, bbox_inches='tight')
    plt.close(fig1)
    print(f"✓ Poses comparison plot saved to {output_file_trans}")
    
    # Plot Rotation X (Pitch)
    axes[0].plot(motion_log_2_time_normalized, gt_rel_pitch, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[0].plot(motion_log_2_time_normalized, motion_log_2['rel_pitch'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[0].set_ylabel('Pitch (degrees)', fontsize=10)
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Rotation Y (Yaw)
    axes[1].plot(motion_log_2_time_normalized, gt_rel_yaw, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    axes[1].plot(motion_log_2_time_normalized, motion_log_2['rel_yaw'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    axes[1].set_ylabel('Yaw (degrees)', fontsize=10)
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)
    
    # # Plot Rotation Z (Roll)
    # axes[2].plot(motion_log_2_time_normalized, gt_rel_roll, 'b-', label='MotionLog 1 (Ground Truth)', linewidth=2)
    # axes[2].plot(motion_log_2_time_normalized, motion_log_2['rel_roll'], 'r--', label='MotionLog 2 (UE Path)', linewidth=1.5, alpha=0.7)
    # axes[2].set_ylabel('Roll (degrees)', fontsize=10)
    # axes[2].legend(loc='best')
    # axes[2].grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.tight_layout()
    fig.savefig(output_file_rot, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✓ Poses comparison plot saved to {output_file_rot}")
    # plt.show()

def plot_algo_log(algo_log, output_file='algo_log_plot.png'):
    """Create plot showing AlgoLog results."""
    fig, axes = plt.subplots(6, 1, figsize=(14, 12))
    fig.suptitle('Algorithm Results (AlgoLog)', fontsize=16, fontweight='bold')
    
    # Plot Position X
    axes[0].plot(algo_log['time'], algo_log['vel_x'], 'g-', linewidth=2)
    axes[0].set_ylabel('Velocity X (cm/s)', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    
    # Plot Position Y
    axes[1].plot(algo_log['time'], algo_log['vel_y'], 'g-', linewidth=2)
    axes[1].set_ylabel('Velocity Y (cm/s)', fontsize=10)
    axes[1].grid(True, alpha=0.3)


    # Plot Position Z
    axes[2].plot(algo_log['time'], algo_log['vel_z'], 'g-', linewidth=2)
    axes[2].set_ylabel('Velocity Z (cm/s)', fontsize=10)
    axes[2].grid(True, alpha=0.3)
    
    # Plot Rotation X
    axes[3].plot(algo_log['time'], algo_log['omega_x'], 'g-', linewidth=2)
    axes[3].set_ylabel('Angular Velocity X (rad/s)', fontsize=10)
    axes[3].grid(True, alpha=0.3)
    
    # Plot Rotation Y
    axes[4].plot(algo_log['time'], algo_log['omega_y'], 'g-', linewidth=2)
    axes[4].set_ylabel('Angular Velocity Y (rad/s)', fontsize=10)
    axes[4].grid(True, alpha=0.3)
    
    # Plot Rotation Z
    axes[5].plot(algo_log['time'], algo_log['omega_z'], 'g-', linewidth=2)
    axes[5].set_ylabel('Angular Velocity Z (rad/s)', fontsize=10)
    axes[5].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ AlgoLog plot saved to {output_file}")
    plt.show()

def main():
    # Find the latest log files
    # repo_root = Path(__file__).parent
    repo_root = Path.cwd()
    print(f"🔍 Searching for log files in {repo_root}...")
    
    # Get MotionLog files (ground truth)
    motion_int = 0  # Specify the lab number for GT file
    # Look for GT (ground truth) file
    gt_int = 0
    motion_logs = sorted(list(repo_root.glob(f'results_pnec/rain/Results_MotionLog_*.csv')))
    if not motion_logs:
        print("❌ No MotionLog files found in groundTruths/")
        return

    gt_logs = sorted(list(repo_root.glob(f'groundTruths_pnec/clear/MotionLog_*.csv')))
    # gt_imgs = sorted(list(repo_root.glob(f'groundTruths_pnec/clear/Capture_*.png')))
    # results_imgs = sorted(list(repo_root.glob(f'results_pnec/clear/Results_Capture_*.png')))
    if gt_logs:
        motion_log_1_path = gt_logs[motion_int+2]  # Use the GT file
        # Get other motion logs for comparison
        other_logs = sorted([f for f in motion_logs if 'GT' not in f.name])
        motion_log_2_path = other_logs[motion_int] if other_logs else None
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
    # algo_logs = list(repo_root.glob(f'logs/AlgoLog_lab{motion_int}*.csv'))
    # if not algo_logs:
    #     print("❌ No AlgoLog files found in build/")
        # return
    # else:
    #     algo_log_path = algo_logs[-1]  # Get latest
    #     print(f"Loading algorithm results: {algo_log_path.name}")
    
    # Load data
    motion_df_1 = load_motion_log(motion_log_1_path)
    # algo_df = load_algo_log(algo_log_path)
    
    print(f"\nMotionLog 1 (ground truth) shape: {motion_df_1.shape}")
    # print(f"AlgoLog shape: {algo_df.shape}")
    
    # Create poses comparison plot (MotionLog 1 vs MotionLog 2)
    if motion_log_2_path:
        motion_df_2 = load_motion_log(motion_log_2_path)
        print(f"MotionLog 2 (UE path) shape: {motion_df_2.shape}")
        output_file_rot = repo_root / f'results_pnec/rain/comparison_plot_rot{motion_log_2_path.name[9:-4]}.png'
        output_file_poses = repo_root / f'results_pnec/rain/comparison_plot_trans{motion_log_2_path.name[9:-4]}.png'
        plot_comparison(motion_df_1, motion_df_2, output_file_rot, output_file_poses)
    else:
        print("⚠️  Skipping poses comparison (only one MotionLog file available)")
    
    # Create AlgoLog plot (standalone, no comparison)
    # output_file_algo = repo_root / f'results/algo_log_plot{motion_log_2_path.name[9:-4]}.png'
    # plot_algo_log(algo_df, output_file_algo)

        

    # img1 = cv2.imread(gt_imgs[motion_int], cv2.IMREAD_GRAYSCALE)
    # img2 = cv2.imread(results_imgs[motion_int], cv2.IMREAD_GRAYSCALE)
    # img2 = cv2.resize(img2, img1.shape[::-1])

    # score, _ = ssim(img1, img2, full=True)
    # print(f"SSIM: {score:.4f}")  # 1.0 = identical

if __name__ == '__main__':
    main()
