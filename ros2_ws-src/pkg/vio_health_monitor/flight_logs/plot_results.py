#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

def add_health_background(ax, t_health, health_states, t_min, t_max):
    """Add colored background based on VIO health state."""
    colors = {
        0: ('#81C784', 0.6),  # CONSISTENT - Verde
        1: ('#81D4FA', 0.6),  # POTENTIALLY_CONSISTENT - Azzurro chiaro (In ripresa)
        2: ('#FFB74D', 0.6),  # POTENTIALLY_INCONSISTENT - Arancione (Warning)
        3: ('#E57373', 0.6)   # INCONSISTENT - Rosso (Errore/Emergenza)
    }
    if len(t_health) == 0:
        return
    for i in range(len(t_health) - 1):
        state = int(health_states[i])
        color, alpha = colors.get(state, ('gray', 0.1))
        ax.axvspan(t_health[i], t_health[i+1], facecolor=color, alpha=alpha, linewidth=0)
    state = int(health_states[-1])
    color, alpha = colors.get(state, ('gray', 0.1))
    ax.axvspan(t_health[-1], t_max, facecolor=color, alpha=alpha, linewidth=0)

def myPlot(time, data_list, labels, title, ncols=2, use_tex=False, t_health=None, health_states=None):
    plt.rcParams.update({"text.usetex": use_tex, "font.family": "serif"})
    n = len(data_list)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3.5 * nrows), squeeze=False)
    axes = axes.flatten()
    
    for i in range(n):
        time_plot = time[:len(data_list[i]['vio'])]
        
        axes[i].plot(time_plot, data_list[i]['vio'], 'k-', label='OpenVINS (VIO)', linewidth=1.5)
        
        if 'px4' in data_list[i] and data_list[i]['px4'] is not None:
            axes[i].plot(time_plot, data_list[i]['px4'][:len(time_plot)], 'b--', label='PX4 EKF2', linewidth=1.5)
            
        if 'tactile' in data_list[i] and data_list[i]['tactile'] is not None:
            axes[i].plot(time_plot, data_list[i]['tactile'][:len(time_plot)], 'g-.', label='Tactile Odometry', linewidth=1.5)
        
        if 'ref' in data_list[i] and data_list[i]['ref'] is not None:
            ref_data = data_list[i]['ref']
            if np.isscalar(ref_data):
                axes[i].axhline(y=ref_data, color='r', linestyle='--', label='Reference')
            else:
                axes[i].plot(time_plot, ref_data[:len(time_plot)], 'r--', label='Ground Truth', linewidth=1.2)
        
        axes[i].set_title(labels[i])
        axes[i].set_xlabel(r"$t$ [s]")
        axes[i].grid(True, linestyle='-', alpha=0.3)

        if "Lambda" in labels[i]:
            if t_health is not None and health_states is not None:
                add_health_background(axes[i], t_health, health_states, time_plot[0], time_plot[-1])
            axes[i].axhline(y=350, color='r', linestyle='--', alpha=0.8, label=r'$K_{j,1}$')
            axes[i].axhline(y=450, color='b', linestyle='--', alpha=0.8, label=r'$K_{j,2}$')

        axes[i].legend(loc='best', fontsize='small')
    
    for j in range(n, len(axes)):
        fig.delaxes(axes[j])
        
    fig.suptitle(title, fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    return fig

def load_data_safe(filename):
    if not os.path.exists(filename):
        print(f"[!] File not found: {filename}")
        return None
    try:
        with open(filename, 'r') as f:
            first_line = f.readline()
            second_line = f.readline()
        if ',' in second_line:
            data = np.genfromtxt(filename, delimiter=',', skip_header=1)
        else:
            data = np.genfromtxt(filename, skip_header=1)
        if data.size == 0:
            return None
        if data.ndim == 1:
            data = data.reshape(1, -1)
        return data
    except Exception as e:
        print(f"[!] Error loading {filename}: {e}")
        return None

def apply_ned_to_enu(x, y, z, roll, pitch, yaw):
    """Convert position and orientation from NED to ENU frame."""
    R_ned_to_enu = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])
    pos_ned = np.column_stack((x, y, z))
    pos_enu = np.dot(pos_ned, R_ned_to_enu.T)
    x_enu = pos_enu[:, 0]
    y_enu = pos_enu[:, 1]
    z_enu = pos_enu[:, 2]
    yaw_enu = np.pi / 2.0 - yaw
    yaw_enu = (yaw_enu + np.pi) % (2 * np.pi) - np.pi
    pitch_enu = -pitch
    roll_enu = roll
    return x_enu, y_enu, z_enu, roll_enu, pitch_enu, yaw_enu

def align_trajectory(x_est, y_est, z_est, yaw_est, x_ref, y_ref, z_ref, yaw_ref):
    """Align estimated trajectory to reference using translation and yaw rotation."""
    x_est = x_est - x_est[0] + x_ref[0]
    y_est = y_est - y_est[0] + y_ref[0]
    z_est = z_est - z_est[0] + z_ref[0]
    delta_yaw = yaw_ref[0] - yaw_est[0]
    x_shifted = x_est - x_ref[0]
    y_shifted = y_est - y_ref[0]
    x_est_aligned = x_shifted * np.cos(delta_yaw) - y_shifted * np.sin(delta_yaw) + x_ref[0]
    y_est_aligned = x_shifted * np.sin(delta_yaw) + y_shifted * np.cos(delta_yaw) + y_ref[0]
    yaw_est_aligned = yaw_est + delta_yaw
    return x_est_aligned, y_est_aligned, z_est, yaw_est_aligned

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tex", action="store_true", help="Use LaTeX fonts")
    ap.add_argument("--save", action="store_true", help="Save figures as PNG")
    ap.add_argument("--folder", type=str, default="", help="Folder in old_logs containing the txt files")
    args = ap.parse_args()

    base_path = ""
    if args.folder:
        base_path = os.path.join("../old_logs", args.folder)
        if not os.path.exists(base_path):
            print(f"[!] Directory not found: {base_path}")
            return

    print(f"[*] Loading data files from {'current directory' if not args.folder else base_path}...")
    px4     = load_data_safe(os.path.join(base_path, 'log_px4.txt') if base_path else 'log_px4.txt')
    tactile = load_data_safe(os.path.join(base_path, 'log_tactile.txt') if base_path else 'log_tactile.txt')
    vio     = load_data_safe(os.path.join(base_path, 'log_vio.txt') if base_path else 'log_vio.txt')
    gt      = load_data_safe(os.path.join(base_path, 'log_ground_truth.txt') if base_path else 'log_ground_truth.txt')
    lam     = load_data_safe(os.path.join(base_path, 'log_eigenvalues.txt') if base_path else 'log_eigenvalues.txt')
    health  = load_data_safe(os.path.join(base_path, 'log_health.txt') if base_path else 'log_health.txt')

    if vio is None or gt is None or len(vio) == 0 or len(gt) == 0:
        print("[!] Critical error: missing VIO or ground truth data.")
        return

    # Extract VIO data
    t_vio = vio[:, 0]
    x_vio, y_vio, z_vio = vio[:, 1], vio[:, 2], vio[:, 3]
    roll_vio, pitch_vio, yaw_vio = vio[:, 4], vio[:, 5], np.unwrap(vio[:, 6])

    # Extract ground truth data
    t_gt = gt[:, 0]
    x_gt, y_gt, z_gt = gt[:, 1], gt[:, 2], gt[:, 3]
    roll_gt, pitch_gt, yaw_gt = gt[:, 4], gt[:, 5], np.unwrap(gt[:, 6])

    # Extract PX4 data
    has_px4 = px4 is not None and len(px4) > 0
    if has_px4:
        t_px4 = px4[:, 0]
        x_px4, y_px4, z_px4 = px4[:, 1], px4[:, 2], px4[:, 3]
        roll_px4, pitch_px4, yaw_px4 = px4[:, 4], px4[:, 5], px4[:, 6]
        print("[*] Converting PX4 odometry from NED to ENU...")
        x_px4, y_px4, z_px4, roll_px4, pitch_px4, yaw_px4 = apply_ned_to_enu(
            x_px4, y_px4, z_px4, roll_px4, pitch_px4, yaw_px4)
        yaw_px4 = np.unwrap(yaw_px4)

    # Extract Tactile data
    has_tactile = tactile is not None and len(tactile) > 0
    if has_tactile:
        t_tactile = tactile[:, 0]
        x_tactile, y_tactile, z_tactile = tactile[:, 1], tactile[:, 2], tactile[:, 3]
        roll_tactile, pitch_tactile, yaw_tactile = tactile[:, 4], tactile[:, 5], tactile[:, 6]
        print("[*] Converting Tactile odometry from NED to ENU...")
        x_tactile, y_tactile, z_tactile, roll_tactile, pitch_tactile, yaw_tactile = apply_ned_to_enu(
            x_tactile, y_tactile, z_tactile, roll_tactile, pitch_tactile, yaw_tactile)
        yaw_tactile = np.unwrap(yaw_tactile)

    # Extract health data
    has_health = health is not None and len(health) > 0
    t_health, health_states = None, None
    if has_health:
        t_health = health[:, 0]
        health_states = health[:, 1]
        print(f"[*] Loaded {len(t_health)} VIO health records.")

    # Time alignment
    start_time = min(t_vio[0], t_gt[0])
    t_vio -= start_time
    t_gt  -= start_time
    if has_px4:      t_px4     -= start_time
    if has_tactile:  t_tactile -= start_time
    if has_health:   t_health  -= start_time

    # Interpolate GT and PX4/Tactile onto VIO timestamps
    x_gt_interp     = np.interp(t_vio, t_gt, x_gt)
    y_gt_interp     = np.interp(t_vio, t_gt, y_gt)
    z_gt_interp     = np.interp(t_vio, t_gt, z_gt)
    roll_gt_interp  = np.interp(t_vio, t_gt, roll_gt)
    pitch_gt_interp = np.interp(t_vio, t_gt, pitch_gt)
    yaw_gt_interp   = np.interp(t_vio, t_gt, yaw_gt)

    if has_px4:
        x_px4_interp     = np.interp(t_vio, t_px4, x_px4)
        y_px4_interp     = np.interp(t_vio, t_px4, y_px4)
        z_px4_interp     = np.interp(t_vio, t_px4, z_px4)
        roll_px4_interp  = np.interp(t_vio, t_px4, roll_px4)
        pitch_px4_interp = np.interp(t_vio, t_px4, pitch_px4)
        yaw_px4_interp   = np.interp(t_vio, t_px4, yaw_px4)

    if has_tactile:
        x_tactile_interp     = np.interp(t_vio, t_tactile, x_tactile)
        y_tactile_interp     = np.interp(t_vio, t_tactile, y_tactile)
        z_tactile_interp     = np.interp(t_vio, t_tactile, z_tactile)
        roll_tactile_interp  = np.interp(t_vio, t_tactile, roll_tactile)
        pitch_tactile_interp = np.interp(t_vio, t_tactile, pitch_tactile)
        yaw_tactile_interp   = np.interp(t_vio, t_tactile, yaw_tactile)

    # Spatial alignment
    print("[*] Aligning trajectories spatially...")
    x_vio, y_vio, z_vio, yaw_vio = align_trajectory(
        x_vio, y_vio, z_vio, yaw_vio,
        x_gt_interp, y_gt_interp, z_gt_interp, yaw_gt_interp)

    if has_px4:
        x_px4_interp, y_px4_interp, z_px4_interp, yaw_px4_interp = align_trajectory(
            x_px4_interp, y_px4_interp, z_px4_interp, yaw_px4_interp,
            x_gt_interp, y_gt_interp, z_gt_interp, yaw_gt_interp)

    if has_tactile:
        x_tactile_interp, y_tactile_interp, z_tactile_interp, yaw_tactile_interp = align_trajectory(
            x_tactile_interp, y_tactile_interp, z_tactile_interp, yaw_tactile_interp,
            x_gt_interp, y_gt_interp, z_gt_interp, yaw_gt_interp)


    # Compute errors (VIO vs Ground Truth)
    err_x      = np.abs(x_vio - x_gt_interp)
    err_y      = np.abs(y_vio - y_gt_interp)
    err_z      = np.abs(z_vio - z_gt_interp)
    err_pos_3d = np.sqrt(err_x**2 + err_y**2 + err_z**2)

    print(f"[*] Mean 3D error:  {np.mean(err_pos_3d):.4f} m")
    print(f"[*] Max  3D error:  {np.max(err_pos_3d):.4f} m")
    print(f"[*] Final 3D error: {err_pos_3d[-1]:.4f} m")

    # Also compute PX4/Tactile errors if available
    if has_px4:
        err_px4_3d = np.sqrt(
            (x_px4_interp - x_gt_interp)**2 +
            (y_px4_interp - y_gt_interp)**2 +
            (z_px4_interp - z_gt_interp)**2)
        print(f"[*] PX4 Mean 3D error:  {np.mean(err_px4_3d):.4f} m")
        print(f"[*] PX4 Max  3D error:  {np.max(err_px4_3d):.4f} m")
        print(f"[*] PX4 Final 3D error: {err_px4_3d[-1]:.4f} m")

    if has_tactile:
        err_tactile_x = np.abs(x_tactile_interp - x_gt_interp)
        err_tactile_y = np.abs(y_tactile_interp - y_gt_interp)
        err_tactile_z = np.abs(z_tactile_interp - z_gt_interp)
        err_tactile_3d = np.sqrt(err_tactile_x**2 + err_tactile_y**2 + err_tactile_z**2)
        print(f"[*] Tactile Mean 3D error:  {np.mean(err_tactile_3d):.4f} m")
        print(f"[*] Tactile Max  3D error:  {np.max(err_tactile_3d):.4f} m")
        print(f"[*] Tactile Final 3D error: {err_tactile_3d[-1]:.4f} m")

    # Figure 1: Position tracking
    fig_pos_data = [
        {'vio': x_vio, 'ref': x_gt_interp, 'px4': x_px4_interp if has_px4 else None, 'tactile': x_tactile_interp if has_tactile else None},
        {'vio': y_vio, 'ref': y_gt_interp, 'px4': y_px4_interp if has_px4 else None, 'tactile': y_tactile_interp if has_tactile else None},
        {'vio': z_vio, 'ref': z_gt_interp, 'px4': z_px4_interp if has_px4 else None, 'tactile': z_tactile_interp if has_tactile else None},
    ]
    myPlot(t_vio, fig_pos_data, ["X [m]", "Y [m]", "Z [m]"],
           "Position Tracking (ENU Frame)", ncols=3, use_tex=args.tex,
           t_health=t_health, health_states=health_states)

    # Figure 2: Orientation tracking
    fig_rpy_data = [
        {'vio': roll_vio,  'ref': roll_gt_interp,  'px4': roll_px4_interp  if has_px4 else None, 'tactile': roll_tactile_interp if has_tactile else None},
        {'vio': pitch_vio, 'ref': pitch_gt_interp, 'px4': pitch_px4_interp if has_px4 else None, 'tactile': pitch_tactile_interp if has_tactile else None},
        {'vio': yaw_vio,   'ref': yaw_gt_interp,   'px4': yaw_px4_interp   if has_px4 else None, 'tactile': yaw_tactile_interp if has_tactile else None},
    ]
    myPlot(t_vio, fig_rpy_data, ["Roll [rad]", "Pitch [rad]", "Yaw [rad]"],
           "Orientation Tracking", ncols=3, use_tex=args.tex,
           t_health=t_health, health_states=health_states)

    # Figure 3: Eigenvalues
    if lam is not None and len(lam) > 0 and lam.shape[1] >= 4:
        t_lam = lam[:, 0] - start_time
        lx, ly, lz = lam[:, 1], lam[:, 2], lam[:, 3]
        fig_eig_data = [{'vio': lx}, {'vio': ly}, {'vio': lz}]
        myPlot(t_lam, fig_eig_data, ["Lambda X", "Lambda Y", "Lambda Z"],
               "OpenVINS Degeneracy Eigenvalues", ncols=3, use_tex=args.tex,
               t_health=t_health, health_states=health_states)

    # Figure 4: X-Y trajectory top view
    fig_xy = plt.figure(figsize=(10, 8))
    plt.plot(x_gt_interp, y_gt_interp, 'r--', label='Ground Truth', linewidth=2)
    plt.plot(x_vio,       y_vio,       'k-',  label='OpenVINS (VIO)', linewidth=2)
    if has_px4:
        plt.plot(x_px4_interp, y_px4_interp, 'b--', label='PX4 EKF2', linewidth=1.5)
    if has_tactile:
        plt.plot(x_tactile_interp, y_tactile_interp, 'g-.', label='Tactile Odometry', linewidth=1.5)
        
    plt.plot(x_gt_interp[0], y_gt_interp[0], 'ro', label='Start Point', markersize=8)
    plt.title("X-Y Trajectory Comparison (Top View)")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # Figure 5: VIO vs Tactile tracking errors
    fig_err_data = [
        {'vio': err_x, 'tactile': err_tactile_x if has_tactile else None},
        {'vio': err_y, 'tactile': err_tactile_y if has_tactile else None},
        {'vio': err_z, 'tactile': err_tactile_z if has_tactile else None},
        {'vio': err_pos_3d, 'tactile': err_tactile_3d if has_tactile else None},
    ]
    myPlot(t_vio, fig_err_data,
           ["Abs Error X [m]", "Abs Error Y [m]", "Abs Error Z [m]", "Total 3D Error [m]"],
           "Absolute Tracking Errors (VIO vs Tactile)", ncols=2, use_tex=args.tex,
           t_health=t_health, health_states=health_states)

    # Figure 6: PX4/Tactile vs VIO error comparison
    if has_px4 or has_tactile:
        fig_cmp, ax_cmp = plt.subplots(figsize=(10, 5))
        ax_cmp.plot(t_vio, err_pos_3d, 'k-',  label='OpenVINS 3D Error', linewidth=1.5)
        if has_px4:
            ax_cmp.plot(t_vio, err_px4_3d, 'b--', label='PX4 EKF2 3D Error', linewidth=1.5)
        if has_tactile:
            ax_cmp.plot(t_vio, err_tactile_3d, 'g-.', label='Tactile Odometry 3D Error', linewidth=1.5)
            
        if t_health is not None and health_states is not None:
            add_health_background(ax_cmp, t_health, health_states, t_vio[0], t_vio[-1])
        ax_cmp.set_title("3D Tracking Error Comparison")
        ax_cmp.set_xlabel(r"$t$ [s]")
        ax_cmp.set_ylabel("3D Error [m]")
        ax_cmp.grid(True, linestyle='-', alpha=0.3)
        ax_cmp.legend(loc='best', fontsize='small')

    if args.save:
        for i in plt.get_fignums():
            plt.figure(i).savefig(f"plot_fig_{i}.png", dpi=300, bbox_inches='tight')
        print("[*] Plots saved as PNG files.")
    else:
        print("[*] Displaying plots...")
        plt.show()

if __name__ == "__main__":
    main()
