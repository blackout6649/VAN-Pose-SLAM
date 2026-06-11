# VAN-Pose-SLAM

Vision-Aided Navigation Pose SLAM Assignment (Homework #4)

## Quick Start

### Prerequisites
- **Conda** (Miniforge or Anaconda) installed on your system
- Git

### Setup (One-time)

1. Clone the repository:
   ```bash
   git clone <repo-url>
   cd VAN-Pose-SLAM
   ```

2. Create the conda environment from the provided file:
   ```bash
   conda env create -f environment.yml
   ```

3. Activate the environment:
   ```bash
   conda activate van-gtsam
   ```

### Run the Assignment

Execute the main script:
```bash
python run_gtsam_python.py
```

Expected output:
```
GTSAM is working.
Loaded 50 initial poses
Loaded 50 ground-truth poses
Loaded 49 odometry measurements
First pose translation: [ -59.25009372 -103.24300182   28.25299061]
```

## Environment Details

The `environment.yml` file specifies:
- **Python 3.11** (compatible with precompiled GTSAM wheels)
- **GTSAM 4.2.1** (Graph-based SLAM library)
- **SciPy, NumPy** (Scientific computing)
- **Matplotlib** (Visualization)

All dependencies are installed automatically via conda without requiring compilation.

## Files

- `environment.yml` — Conda environment specification (reproducible across machines)
- `run_gtsam_python.py` — Python script demonstrating GTSAM usage with assignment data
- `data.mat` — Provided dataset (odometry, initial trajectory, ground truth)
- `GTSAM.m` — Original MATLAB starter script (reference only)
- `gtsam-4.2.1/` — GTSAM source code (reference; not needed for execution)

## Notes

- The assignment was originally intended for MATLAB, but Python with GTSAM is functionally equivalent and avoids compilation issues.
- If you need to restore conda SSL verification after setup:
  ```bash
  conda config --set ssl_verify true
  ```

## Troubleshooting

**Q: Conda environment creation fails?**
A: Ensure you are connected to the internet and your network proxy settings are correct (if behind a corporate proxy).

**Q: GTSAM import error?**
A: Verify you have activated the correct conda environment:
```bash
conda activate van-gtsam
python -c "import gtsam; print('OK')"
```

**Q: Data file not found?**
A: Make sure `data.mat` is in the same directory as `run_gtsam_python.py`.
