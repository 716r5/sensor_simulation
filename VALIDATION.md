# Software validation

The supplied code was checked in Python on 23 October 2026. These checks passed:

- Pico/PC binary record interoperability; corruption of each individual record byte was rejected by CRC.
- BMP180 manufacturer worked example: UT=27898, UP=23843 produced 15 C and 69964 Pa.
- Simulated byte-by-byte power interruption during FRAM append, including the final commit byte; earlier committed slots remained byte-for-byte unchanged, and reboot allocation did not overwrite them.
- Full simulated FRAM was rejected without modifying it.
- Constant-signal, linear-trend, and seeded white-noise Allan/Hadamard numerical identities.
- Synthetic level, roll, and pitch trajectories; acceleration rejection and positive-definite EKF covariance.
- An end-to-end synthetic capture with known accelerometer offsets/scales and gyro biases: calibration recovered those parameters within the test tolerance, and analysis produced calibrated CSV, EKF plots, variance table, and stability plots.
- Python syntax compilation of all source files.

Not verified: physical sensor response, actual MicroPython runtime compatibility on your firmware, USB performance on your computer, actual sampling jitter, physical FRAM bus writes, flash endurance/power behavior, electrical brownout/reset behavior, absolute measurement accuracy, or flight dynamics. There are no real hardware recordings in this package.

Do the check.py, independent calibration validation, and controlled recovery tests in README.md before relying on the setup. Long-term unattended or mission-critical retention needs adequately sized storage and an electrically validated power-fail design.
