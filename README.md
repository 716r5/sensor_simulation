Sensor software simulation update

BMP180  |  BMI160  |  ADXL345 | Raspberyy Pi Pico
Ran the processing checks in Python using simulated data before connecting the sensors. Calibration, Allan and Hadamard variance, a tilt EKF and the logging format have been tested in software. Proper hardware testing is still pending - issues with the board, unforunately. Hence, I simulated the circuits via code.

**Calibration**
I generated six stationary orientations for each accelerometer with known offsets, scale errors and random noise. Each face had 300 samples. Fitted the offsets and scales, then checked them on a separate 600 sample stationary recording.
The maximum recovered offset error was 0.00008 g for the BMI160 model and 0.00016 g for the ADXL345 model. 
The corrected mean acceleration norms were 1.0001 g and 1.0004 g.
For the gyro model, subtracting a separately estimated bias left a maximum stationary axis mean of 0.0024 degrees/s.
The BMP180 compensation code also reproduced the datasheet example: raw temperature 27898 and pressure 23843 gave 15 degrees C and 69964 Pa. This checks the calculation, not the actual sensor accuracy.
**Allan and Hadamard variance**
I used 18,000 generated samples at 10 Hz, equivalent to 30 minutes. 
The noise standard deviation was 0.2 degrees/s. A second signal used the same noise plus a linear drift of 0.001 degrees/s per second. Both estimators use overlapping averages.
At 0.1 s, Allan variance was 0.04049 and Hadamard variance was 0.04048 (degrees/s) squared, close to the input noise variance of 0.04000. Constant input gave zero for both. A pure linear ramp gave near-zero Hadamard variance while Allan variance increased with averaging time.
**EKF**
I tested roll and pitch separately with a 20 degree sinusoidal reference over 60 seconds at 10 Hz. Added acceleration noise of 0.01 g and gyro noise of 0.2 degrees/s. The roll RMSE was 0.208 degrees and pitch RMSE was 0.168 degrees. Covariance stayed positive for both runs , roll and pitch.
The EKF uses gyro prediction and gravity correction from the BMI160 model. Yaw and position are not estimated yet - i will do test these again.

**Logging and recovery checks** - to ensure the data doesnt suddenly disappear
So far have tested a 72 byte binary record containing a boot ID, sequence number, timestamp, raw values, compensated pressure and temperature, and CRC32. 
All 72 single-byte corruption cases were rejected! which is good. The Pico and PC record decoders returned the same values.
For local retention, I used a 32 KB FRAM memory model with 80 byte slots (to retest with different values). I interrupted the second record write after each possible byte count, then simulated a restart. Note: They do not prove survival of real brownouts, faults or storage damage. The 32 KB model holds 409 records, about 41 seconds at 10 Hz, so longer standalone recording needs more storage.

