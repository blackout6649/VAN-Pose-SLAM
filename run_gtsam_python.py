import numpy as np
import scipy.io as sio
import gtsam
from gtsam import symbol


def _matlab_cell_to_list(cell_array):
    """Convert MATLAB cell array loaded by scipy.io.loadmat into a Python list."""
    flat = np.ravel(cell_array)
    return [np.array(item) for item in flat]


def main():
    data = sio.loadmat("data.mat")
    traj3 = _matlab_cell_to_list(data["traj3"])
    poses3_gt = _matlab_cell_to_list(data["poses3_gt"])
    dpose = _matlab_cell_to_list(data["dpose"])

    values = gtsam.Values()
    for i, T in enumerate(traj3):
        values.insert(symbol("x", i), gtsam.Pose3(T))

    print("GTSAM is working.")
    print(f"Loaded {len(traj3)} initial poses")
    print(f"Loaded {len(poses3_gt)} ground-truth poses")
    print(f"Loaded {len(dpose)} odometry measurements")

    p0 = values.atPose3(symbol("x", 0))
    print("First pose translation:", p0.translation())


if __name__ == "__main__":
    main()
