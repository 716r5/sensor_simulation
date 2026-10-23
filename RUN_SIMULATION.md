# Reproduce the report

Run this on your laptop in normal Python, using VS Code's terminal. No Pico, sensors or Thonny connection is needed.

1. Unzip the folder and open it in VS Code.
2. Open Terminal > New Terminal.
3. Run these Windows commands:

```powershell
cd pc
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements_simulation.txt
.\.venv\Scripts\python.exe run_simulation.py
```

If `py` is not found, install normal Python 3 on the laptop first. Python installed inside Thonny does not necessarily provide the `py` command. Python 3.12 was used for these simulations.

Outputs are saved in `pc/simulation_results`:

- `simulation_plots.png`: the four plots in the Word report.
- `simulation_metrics.json`: numerical results before report rounding.
- `tilt_simulation.csv`: reference angles and EKF estimates.
- `variance_simulation.csv`: Allan/Hadamard variance values.
- `calibration_inputs/`: generated six-face and stationary data.

The script uses seed 42 and the same operations that generated the report. It checks its results against `pc/expected_simulation_metrics.json` and runs the software checks for BMP180 compensation, CRC, FRAM interruption, variance identities and EKF behavior. Expect `PASS: simulation metrics match the report reference`.

Rounded report values:

| Result | Value |
|---|---:|
| BMI160 model maximum accelerometer offset error | 0.00008 g |
| ADXL345 model maximum accelerometer offset error | 0.00016 g |
| BMI160 corrected mean acceleration norm | 1.0001 g |
| ADXL345 corrected mean acceleration norm | 1.0004 g |
| Gyro model maximum stationary axis mean after calibration | 0.0024 degrees/s |
| Allan variance at 0.1 s | 0.04049 (degrees/s) squared |
| Hadamard variance at 0.1 s | 0.04048 (degrees/s) squared |
| Roll RMSE | 0.208 degrees |
| Pitch RMSE | 0.168 degrees |

Small differences in the last decimal digits or plot fonts can occur across numerical libraries and operating systems. The reported rounded values should match with the supplied versions and unchanged code. Changing the random seed, noise, sample rate or trajectory changes the results. Real sensor measurements will differ.

`run_simulation.py` recreates the report's figures and metrics; it does not rewrite the Word report. Repeated runs replace only generated outputs in `simulation_results`. Keep real recordings elsewhere.

The earlier README describes hardware setup and analysis of actual captures. Start with this file for reproducing the software report.
