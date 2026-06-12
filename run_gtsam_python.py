import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gtsam
from gtsam import symbol


def _matlab_cell_to_list(cell_array):
    """Convert MATLAB cell array loaded by scipy.io.loadmat into a Python list."""
    flat = np.ravel(cell_array)
    return [np.array(item) for item in flat]


def plot_trajectory(trajectory, poses3_gt, est_label="Estimated Trajectory", gt_label="Ground Truth"):
    """Plot 3D trajectory comparing estimated poses and ground truth """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Extract positions from estimated trajectory
    est_positions = np.array([
        trajectory.atPose3(symbol("x", i)).translation() 
        for i in range(trajectory.size())
    ])

    # Extract positions from ground truth
    gt_positions = np.array([
        gtsam.Pose3(T).translation() 
        for T in poses3_gt
    ])

    ax.plot(est_positions[:, 0], est_positions[:, 1], est_positions[:, 2], 
            "r-", label=est_label, linewidth=2)
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], 
            "g--", label=gt_label, linewidth=2)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=0, azim=-90)
    ax.set_title("3D Pose SLAM Trajectory")
    ax.legend()
    plt.show()


def build_factor_graph(initial_trajectory, dpose):
    """Build the Pose SLAM factor graph using the initial trajectory as a prior estimate."""
    graph = gtsam.NonlinearFactorGraph()
    noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([1e-3, 1e-3, 1e-3, 0.1, 0.1, 0.1])
    )

    first_key = symbol("x", 0)
    graph.add(gtsam.PriorFactorPose3(first_key, gtsam.Pose3(), noise_model))

    for i, relative_transform in enumerate(dpose):
        key1 = symbol("x", i)
        key2 = symbol("x", i + 1)
        relative_pose = gtsam.Pose3(relative_transform)
        graph.add(gtsam.BetweenFactorPose3(key1, key2, relative_pose, noise_model))

    return graph, noise_model


def optimize_trajectory(graph, initial_estimate):
    """Run MAP optimization for the Pose SLAM problem."""
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    return optimizer.optimizeSafely()


def compute_localization_error(result, poses3_gt):
    """Compute localization error """
    n = result.size()
    errors = np.array([
        np.linalg.norm(
            result.atPose3(symbol("x", i)).translation() - 
            gtsam.Pose3(poses3_gt[i]).translation()
        )
        for i in range(n)
    ])
    return errors


def plot_localization_error(result_without_loop, result_with_loop, poses3_gt):
    """Plot localization error over time comparing with and without loop closure."""
    errors_without = compute_localization_error(result_without_loop, poses3_gt)
    errors_with = compute_localization_error(result_with_loop, poses3_gt)
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ax.plot(errors_without, "b-", label="Without Loop Closure", linewidth=2, marker="o", markersize=4)
    ax.plot(errors_with, "r-", label="With Loop Closure", linewidth=2, marker="s", markersize=4)
    
    ax.set_xlabel("Pose Index")
    ax.set_ylabel("Localization Error (meters)")
    ax.set_title("Localization Error Analysis: Impact of Loop Closure")
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    

def plot_marginals(graph, result, poses3_gt):
    """Plot MAP trajectory with marginal position uncertainty and ground truth."""
    marginals = gtsam.Marginals(graph, result)
    n = result.size()

    positions = np.array([
        result.atPose3(symbol("x", i)).translation() for i in range(n)
    ])
    gt_positions = np.array([
        gtsam.Pose3(T).translation() for T in poses3_gt
    ])

    # Translation block is rows/cols 3:6 of the 6x6 Pose3 covariance (rotation first in GTSAM)
    pos_std = np.array([
        np.sqrt(np.diag(marginals.marginalCovariance(symbol("x", i))[3:6, 3:6]))
        for i in range(n)
    ])

    fig = plt.figure(figsize=(12, 5))

    ax = fig.add_subplot(121, projection="3d")
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            "b-", label="MAP Trajectory", linewidth=2)
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2],
            "g--", label="Ground Truth", linewidth=2)
    ax.view_init(elev=0, azim=-90)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("MAP Trajectory (ZX view)")
    ax.legend()

    ax2 = fig.add_subplot(122)
    ax2.plot(pos_std[:, 0], label="σ_x")
    ax2.plot(pos_std[:, 1], label="σ_y")
    ax2.plot(pos_std[:, 2], label="σ_z")
    ax2.set_xlabel("Pose index")
    ax2.set_ylabel("Position std (m)")
    ax2.set_title("Marginal Position Uncertainty (1σ)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    # Load data from MATLAB .mat file
    data = sio.loadmat("data.mat")
    traj3 = _matlab_cell_to_list(data["traj3"])         # Initial estimated trajectory (list of 4x4 transformation matrices)
    poses3_gt = _matlab_cell_to_list(data["poses3_gt"]) # Ground truth poses
    dpose = _matlab_cell_to_list(data["dpose"])         # Noisy relative pose measurements

    trajectory = gtsam.Values() # Create an empty Values object to hold the trajectory
    for i, T in enumerate(traj3):
        trajectory.insert(symbol("x", i), gtsam.Pose3(T)) # Insert each Pose3 into the Values object with a unique key

    # Plot the trajectory and ground truth
    plot_trajectory(trajectory, poses3_gt, est_label="Initial Estimated Trajectory", gt_label="Ground Truth") 

    graph, noise_model = build_factor_graph(trajectory, dpose)
    result = optimize_trajectory(graph, trajectory)

    plot_trajectory(result, poses3_gt, est_label="MAP Trajectory (Without Loop Closure)", gt_label="Ground Truth")

    plot_marginals(graph, result, poses3_gt)

    # Loop closure
    R21 = gtsam.Rot3(np.array([
    [0.330571768,  0.0494690228, -0.942483486],
    [0.0138000518, 0.998265226,   0.0572371968],
    [0.943679959,  -0.0319273223,  0.329315626]
    ]))
    t21 = gtsam.Point3(-24.1616858, -0.0747429903, 275.434963)
    loop21 = gtsam.Pose3(R21, t21)
    # Add loop closure factor between x(t1=3) and x(t2=42)
    graph.add(gtsam.BetweenFactorPose3(symbol("x", 3), symbol("x", 42), loop21, noise_model))

    # Re-optimize with loop closure
    result_with_loop = optimize_trajectory(graph, trajectory)

    plot_trajectory(result_with_loop, poses3_gt, est_label="MAP Trajectory (With Loop Closure)", gt_label="Ground Truth")
    plot_marginals(graph, result_with_loop, poses3_gt)
    
    # Localization error
    plot_localization_error(result, result_with_loop, poses3_gt)
if __name__ == "__main__":
    main()
