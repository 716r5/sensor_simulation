# Pico + BMP180 + BMI160 + ADXL345: a staged sensor project

Implements sensor identity/configuration checks, raw acquisition, checksummed logging, six-face accelerometer calibration, stationary gyro bias calibration, overlapping Allan/Hadamard variance, and a nonlinear roll/pitch EKF. The code has software simulations and numerical tests.

## 1. What this estimates

| Sensor | Data | Use in this project |
|---|---|---|
| BMP180 | Pressure and temperature | Factory compensation, optional pressure offset, relative altitude, stability analysis |
| BMI160 | 3-axis acceleration and angular rate | Calibrated acceleration/rate, roll/pitch EKF, stability analysis |
| ADXL345 | 3-axis acceleration | Independent acceleration comparison and stability analysis |

Allan variance describes how noise changes with averaging time. Hadamard variance uses a second difference of averaged readings and rejects a linear trend in the measured signal. Neither is a calibration or a real-time smoothing filter.

The EKF intentionally estimates only roll/pitch using the BMI160. BMP180 pressure does not measure tilt, so it is not forced into the tilt filter. ADXL345 is retained as an independent check; it is not blindly averaged with the BMI160. There is no absolute yaw, global position, or flight trajectory estimate. A barometer/IMU altitude EKF needs a separate model, mounting definition, dynamics, and validation. This is a slow bench demonstration, not flight or control software.

## 2. The power-loss requirement

**A sample read into RAM is not yet a saved sample. A write in progress can be lost. No system can promise survival of every possible electrical fault or physical destruction.** The practical requirement should be: after an ordinary sudden loss of supply, every previously completed/verified durable record remains recoverable; an interrupted last record is detected and ignored. Acquisition cannot continue while the system has no power.

The included modes are deliberately explicit:

| MODE in pico/main.py | Local storage | Appropriate use | Limitation |
|---|---|---|---|
| `flash` (default) | Pico internal flash | 60-second initial demonstration with existing parts | Flush + sync per record; no unconditional brownout guarantee, limited space/endurance |
| `stream` | Laptop disk through USB | Long stationary data collection with the laptop connected | Pico stores nothing locally in this mode; USB/PC failure loses subsequent samples |
| `fram` | Added 32 KiB I2C FRAM | Standalone retention/power-cut demonstration | Only 409 complete records, about 40.9 seconds at 10 Hz; no auto-overwrite |

Every binary record contains boot/session ID, sequence number, microsecond elapsed timestamp, raw readings, compensated pressure/temperature, and a CRC32. The raw BMP180 UT/UP are retained as well. The Pico prints a record only after its selected storage operation succeeds. In stream mode it is only transmitted, not locally committed. The laptop retains received wire bytes and fsyncs each accepted CSV row. A console line alone is not proof of physical durable storage.

Flash uses a new filename on every start. It never opens an old file for writing, never deletes old data, and stops near full capacity. LittleFS is designed for power-loss resilience, but actual behavior still depends on the firmware filesystem, flash, and supply. Do not reflash firmware, format, run flash_nuke, or delete files during recovery. Export first. Frequent sync operations consume flash endurance; do not use flash mode for extended logging.

FRAM uses unused fixed slots, a checksum, payload readback, and a final commit byte written separately and read back. Startup scans slots instead of rewriting a shared allocation counter. Incomplete slots are not reused; valid old slots are never modified. When full, recording stops. CRC detects corruption with a very small residual collision probability; it cannot repair data or protect against an arbitrary misaddressed write under invalid voltage.

### Hardware for data that really matters

Use FRAM/MRAM as the primary journal, a supply supervisor that holds the Pico in reset before voltage falls outside *either* the processor or memory specification, appropriate write protection, and a correctly designed isolated hold-up supply. Write protection must be asserted early enough, with datasheet-compliant timing; a write-protect pin or capacitor alone is not a complete power-fail circuit. Use short, secure connections for the final assembly. A breadboard/jumper setup is appropriate for checks, not reliable vibration/flight logging.

For long recordings, use sufficiently large FRAM, or FRAM as a write-ahead journal with an SD archival copy. **The SD write-ahead/reclaim design is not implemented here.** Do not reclaim FRAM merely because a FAT file readback succeeded: an SD controller can still have volatile state. Either retain primary FRAM data for the whole mission or use storage with a documented power-fail contract and validate the complete system. An ordinary microSD plus `flush()` does not meet an absolute retention claim. A second physically separate copy improves recovery.

This version uses 80-byte FRAM slots. Required primary capacity is `80 × 10 × seconds` bytes, plus margin: 30 minutes needs 1,440,000 bytes (about 1.37 MiB); 1 hour needs 2,880,000 bytes. Do not buy a 32 KB module expecting hours of storage. A larger chip may need different SPI/addressing code, so changing a capacity constant is not enough.

## 3. Wire the sensors

Disconnect USB/power first. These instructions assume **breakout modules with headers**, not bare sensor ICs. If the Pico has no headers, solder headers first. You also need a USB **data** cable. Keep wires short.

| Pico signal | Physical Pico pin | Connect on all three breakout modules |
|---|---:|---|
| 3V3(OUT) | 36 | 3.3 V supply input, if supported by that exact breakout |
| GND | 38 (or another GND) | GND |
| GP0, I2C0 SDA | 1 | SDA |
| GP1, I2C0 SCL | 2 | SCL |

All three share the same SDA/SCL lines and ground through the breadboard rails. Do not use VBUS/5 V for these connections. Pico GPIO is not 5 V tolerant. Check the regulator arrangement: some modules label the input VIN and require something different from their 3V3 pin. Send front/back photographs if the supply pin labeling is unclear. Check whether breadboard power rails are split in the middle.

For I2C mode, ADXL345 CS must be high to 3.3 V and SDO/ALT ADDRESS low for address 0x53. BMI160 CS/CSB must also be high, with SDO low for 0x68; SDO high gives 0x69. Do not override breakout straps without checking their schematic. BMP180 normally uses 0x77. These addresses do not conflict. The driver accepts ADXL345 0x1D as well.

Most breakouts already have pull-ups. If yours do not, add one ~4.7 kohm SDA-to-3.3 V resistor and one SCL-to-3.3 V resistor. Do not keep adding pull-ups on every module; parallel pull-ups can become too strong. Never pull these lines up to 5 V.

Optional supported FRAM: **MB85RC256V, 32 KiB I2C**, at 0x50, powered at 3.3 V, sharing SDA/SCL/GND. Set address straps A0/A1/A2 low and WP low for writes, according to the breakout schematic. WP low enables this bench code; it does not implement hardware brownout protection. EEPROM modules such as AT24C256 are NOT drop-in replacements for this code.

Mount both accelerometers rigidly for comparison. Their printed PCB orientations may differ from their actual sensor axes. Determine each sensor's +X/+Y/+Z from its documentation and response. Their axes need not be aligned for independent norm plots; separate six-face files are supported. Do not compare individual axis values until you define a rotation between the frames.

## 4. Where to run the code

Use **Thonny for the Pico** and **VS Code's terminal for laptop analysis**. MicroPython runs on the Pico; NumPy/Matplotlib run on your laptop. Do not try to install NumPy on the Pico or run `machine.I2C` in normal Windows Python.

1. Install Thonny from https://thonny.org/ and normal Python 3 from https://www.python.org/. On a company-managed device, use your approved software-installation process.
2. Download the official MicroPython UF2 for your exact Pico model from https://www.raspberrypi.com/documentation/microcontrollers/micropython.html . Pico, Pico W, and Pico 2 firmware differ.
3. Hold BOOTSEL while connecting the Pico by USB. Copy the UF2 to RPI-RP2. The Pico restarts. This is first-time installation; do not reinstall firmware as a routine way to recover logs.
4. In Thonny select the MicroPython (Raspberry Pi Pico) interpreter and its USB serial port. The exact menu label depends on your version.
5. Open `pico/sensors.py` and `pico/records.py`. Save each to **Raspberry Pi Pico**, with exactly those filenames, using Save As. They must be in the Pico filesystem root, not merely on the laptop.
6. Open `pico/check.py` in Thonny and run it. Start here before copying `main.py`.
7. After checks pass, save `pico/export.py` and finally `pico/main.py` to the Pico root. A file called `main.py` auto-runs when the Pico starts. Press Ctrl+C in Thonny to stop it.
8. Unzip this project on the laptop. In VS Code open the project folder and a terminal. Run the commands below from the **pc** folder. Windows examples use `py`; use `python` or `python3` if that is your normal command.

```powershell
cd pc
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

No PowerShell activation is needed. This avoids changing execution policy. For macOS/Linux, create the same environment and use `.venv/bin/python` instead of the Windows interpreter path.

## 5. Confirm all sensors work before processing

Run `check.py` in Thonny. Expect 0x77, 0x68 (or 0x69), and 0x53 (or 0x1D). IDs checked: BMP180 0x55, BMI160 0xD1, ADXL345 0xE5. An I2C address or chip ID proves communication, not sensor health.

1. On a still desk, each accelerometer should have a vector magnitude close to 1 g. BMI160 raw counts are divided by 16384; ADXL345 counts are multiplied by 0.0039 g. A vertical axis is approximately +1 g or -1 g; other axes are approximately zero.
2. Turn each sensor through all six faces. The corresponding axis should change sign and approach +/-1 g, without stuck values. Do not expect exact values before calibration.
3. Rotate BMI160 gently about each axis. Its matching gyro axis should respond, then return near its stationary bias. Divide gyro counts by 65.6 to obtain degrees/s with this configured +/-500 degrees/s range.
4. BMP180 should give plausible ambient temperature and pressure. Factory compensation is already applied. Raise/lower it gently by a few metres if possible; pressure should decrease with height. Do not blow into it or touch its sensing port. Desk-level pressure changes may be small compared with short-term noise.
5. If a device is missing, disconnect power and connect **one module at a time**, checking rails, grounds, CS/address straps, soldering, and cable before changing software.

This project checks IDs, BMI160 operating/configuration status, fresh inertial data flags, and physical response. It does not implement each manufacturer's electrical self-test procedure or certify sensor health.

## 6. First recording with existing parts

Keep default `MODE = 'flash'` for the first 60-second test. Close/disconnect Thonny before opening the serial port from another program. Find the Pico COM number in Thonny or Device Manager, then replace COM5 below.

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --start --seconds 70
```

`--start` stops the currently running interpreter task and soft-reboots it while capture is ready. It does not erase files. It starts a fresh session, so do not use it in the middle of a recording you intend to keep continuous. The Pico should report a demo-duration stop. Press Ctrl+C to stop the PC capture if needed. The laptop creates a `.wire` file plus one CSV per boot, with unique names. Raw logs also remain on the Pico.

To export retained Pico flash records, keeping Thonny closed:

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --export flash --seconds 90 --out recovered
```

Export stops acquisition, prints retained records, and never deletes them. Allow more than 90 seconds for larger exports and stop with Ctrl+C after output finishes. If you recover logs more than once, copies contain duplicate boot/sequence records. Do not concatenate them and treat duplicates as new measurements. Files are split by boot for analysis.

Alternatively, use Thonny's file browser to copy `log000001.bin` from the Pico to the laptop. Convert a copied binary file or a saved `.wire` capture:

```powershell
.\.venv\Scripts\python.exe work.py convert log000001.bin --out recovered.csv
```

Never open binary files in a text editor and save over them. Recovery rejects truncated or CRC-invalid frames and reports counts. It does not invent missing data. Fixed-length binary recovery assumes no inserted/deleted bytes, as with a torn final write; arbitrary media corruption may need a separate forensic recovery tool.

## 7. Calibration

Warm sensors up for roughly two minutes. Calibration is device-specific, mounting-specific, and temperature-dependent. Do not subtract 1 g from every axis, and do not treat tilt or linear motion as bias.

For calibration and long tests, set `MODE = 'stream'`, save `main.py` to the Pico, then close Thonny. The laptop is now the storage device, so it must remain connected. Use a protected laptop supply and back up captures. For standalone retention use adequately sized nonvolatile memory instead.

Collect a stationary 60-second recording:

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --start --seconds 60 --out still
```

Collect six stable 10-second captures: +X points up, -X up, +Y up, -Y up, +Z up, -Z up. For example:

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --start --seconds 10 --out face_PX
```

Position the sensors and allow motion to settle **before** each capture. Repeat with the corresponding output folder names. Each folder contains a uniquely named CSV. Copy the intended CSVs on your PC into a working `caldata` folder with the short names `still.csv`, `PX.csv`, `NX.csv`, `PY.csv`, `NY.csv`, `PZ.csv`, `NZ.csv`. Keep original captures as well. Renaming/copying a CSV on the PC does not modify Pico logs.

If both sensor coordinate frames are aligned, the same six captures can calibrate both. If not, collect another set orienting the ADXL345's own axes and provide `--adxl-faces` in the same +X,-X,+Y,-Y,+Z,-Z order.

```powershell
.\.venv\Scripts\python.exe work.py calibrate --still caldata/still.csv --faces caldata/PX.csv caldata/NX.csv caldata/PY.csv caldata/NY.csv caldata/PZ.csv caldata/NZ.csv --out calibration.json
```

For each accelerometer axis, `bias=(positive_mean+negative_mean)/2`, `scale=(positive_mean-negative_mean)/2`, and `corrected_g=(measured_g-bias)/scale`. This fits offset and axis scale only, not cross-axis misalignment. Gyro calibration subtracts a stationary mean; gyro scale and temperature calibration need additional references/tests. BMP180 already uses its individual factory coefficients. Its absolute pressure is not independently calibrated unless you supply a trusted reference at the same position/time:

```powershell
# Add this option to the calibrate command only with a real local pressure reference:
--pressure-reference-pa 100500
```

Do not use weather-app sea-level pressure as local station pressure. The saved `p0_pa` is a relative-height baseline, not proof of absolute altitude accuracy. Keep `calibration.json` with the raw recordings; it contains input hashes and calibration values. Use a new output filename for a new calibration; the program refuses to overwrite an existing file.

Validate on a **new** six-face set after calibration. Check each corrected gravity vector and record residuals. Validate the stationary gyro mean on a new capture. Calibration on the same samples used to fit it is not independent verification. Repeat calibration after a mounting change or meaningful temperature change.

## 8. Allan and Hadamard analysis

Record at least 30-60 minutes with the sensors stationary on a solid surface, away from vibrations, fan drafts, handling, and rapid temperature changes. Use stream mode for a laptop test. Start with a short trial to verify timing and disk logging, then run:

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --start --seconds 1800 --out stationary_long
.\.venv\Scripts\python.exe work.py analyze stationary_long/YOUR_CAPTURE.csv --cal calibration.json --out results_still --stationary
```

Replace YOUR_CAPTURE.csv with the actual filename. Do not use the `.wire` file directly for analyze; convert it first if the CSV needs recovery. `results_still` must not already exist. The script verifies consecutive sequences and monotonic timestamps, and refuses this variance calculation if timing jitter exceeds 10%, the recording is too short, or a basic stationarity check fails. It does not silently interpolate gaps. A quiet sensor can still have real drift; sanity checks are not a proof of stationarity.

For equally spaced measured samples y, take m-sample moving means. The implemented overlapping estimators are:

`Allan variance = mean((mean[i+m]-mean[i])**2) / 2`

`Hadamard variance = mean((mean[i+2m]-2*mean[i+m]+mean[i])**2) / 6`

Here tau=m*sample_interval, means are separated by m samples, and each valid starting sample contributes an overlapping term. These are formulas for measured rate/acceleration/pressure samples, not for an integrated angle/time-error series. Variance has squared channel units; plots show its square root, deviation, in channel units. Large tau estimates have less statistical support; overlap term counts are not independent degrees of freedom. No confidence intervals or automatic noise-coefficient fit is claimed.

The logging target is 10 Hz, while inertial sensors run at 25 Hz with their configured internal filters. Register polling and sequential reads do not provide precision synchronization or capture every internal sensor sample. Results characterize this filtered acquisition setup, not the entire sensor noise spectrum. For precision IMU characterization, use FIFO/data-ready acquisition and a deliberate output-rate/filter/decimation design. Do not infer a complete gyro noise model from one short chart or derive all EKF Q/R terms directly from a single variance point.

## 9. EKF demonstration

Make another 30-60 second capture. Start still, tilt gently around roll/pitch within about +/-60 degrees, then return level. Avoid shaking, translation, upside-down positions, and rapid rotations. Run:

```powershell
.\.venv\Scripts\python.exe work.py analyze motion/YOUR_CAPTURE.csv --cal calibration.json --out results_motion
```

The two-state EKF propagates roll/pitch with nonlinear Euler angular-rate equations and corrects using the predicted gravity vector. It uses real logged intervals, a process Jacobian, a gravity measurement Jacobian, innovation gating, and the Joseph covariance update. Gyro bias comes from calibration, not an online bias state. Q/R are clearly marked starter values, not experimentally validated tuning. Euler angles become singular near +/-90 degrees pitch; the script stops near 75 degrees and a quaternion EKF is needed for larger rotations.

An acceleration norm check rejects obvious translational acceleration, but sustained acceleration can still be mistaken for gravity. ADXL345 comparison and BMP180 relative altitude are plotted beside tilt, not claimed as additional tilt observations. For a physical accuracy demonstration, use an angle reference or jig and compare independent known angles with the estimate.

## 10. What to show

1. Photograph of wiring and a captured I2C/ID check.
2. Actual raw `.bin`/`.wire` and CSV recordings, with boot IDs, increasing sequence numbers/timestamps, and CRC acceptance/rejection counts.
3. Your `calibration.json` and a fresh validation set showing residual acceleration/gyro offsets.
4. `calibrated.csv`, preserving raw files separately. Contains calibrated sensor channels, relative altitude, EKF angles, and whether each acceleration update was used.
5. `sensor_and_ekf.png`, `stability_deviation.png`, `variances.csv`, and `summary.json`. Variance files are produced only for accepted stationary data.
6. The controlled power-cut test below, with before/after record counts and hashes.

There are no fabricated hardware results in this package.

## 11. FRAM retention demonstration

Connect the supported 32 KiB FRAM and verify 0x50 appears in an I2C scan. Before its **first use only**, intentionally erase/initialize it from Thonny while main.py is stopped:

```python
from sensors import bus
from records import initialize_fram
initialize_fram(bus(), 'ERASE ALL FRAM DATA')
```

This command destroys all FRAM records, so never run it as recovery or startup code. The logger deliberately does not initialize storage automatically.

Set `MODE = 'fram'`. Save main.py, close Thonny, start a PC capture and let 5-10 seconds record. Interrupt **the Pico supply only** by removing its USB cable; the laptop must stay powered. The capture may exit with a serial error; its synced output remains. Reconnect, then export promptly, because main.py can resume recording into unused FRAM slots:

```powershell
.\.venv\Scripts\python.exe work.py capture --port COM5 --export fram --seconds 30 --out powercut_recovered
```

Recheck COM number if needed. In later tests, stop as soon as enough data has been captured; 409 slots fill quickly and must never be automatically erased. If already full, main.py reports an error, but export still works. Flash/FRAM export does not require the sensors to be working.

For each boot ID, compare every previously accepted pre-cut record with the recovered record having the same boot/sequence. All earlier committed records should match exactly and pass CRC. A final partially written record may be rejected. Repeat at randomly chosen instants, including during writes, over many trials. Also test reconnect/restart, I2C disconnect, and full storage. Keep an evidence table with trial ID, pre-cut committed count, recovered matches, missing/changed records, and rejected partial records. Measure the supply/reset behavior for the final electrical design. **Unplug tests and software simulations alone do not validate all brownouts.**

Use the included comparison command on CSVs for the **same boot ID**. It writes a report, checks all columns, and exits with an error if any earlier row is missing or changed. Extra records after the cut do not cause failure:

```powershell
.\.venv\Scripts\python.exe work.py compare --before logs/PRE_CUT_BOOT.csv --after powercut_recovered/RECOVERED_SAME_BOOT.csv --out powercut_trial01.json
```

The baseline is the rows the laptop actually captured and synced, not every sample that might have existed briefly in Pico RAM. Ensure the laptop capture starts before your test session and stays powered through the cut. Keep the source `.wire`/`.bin` files for independent CRC verification.

## 12. Software verification and sources

Run the included software test on the PC:

```powershell
.\.venv\Scripts\python.exe test_software.py
```

It checks numerical variance identities, synthetic EKF behavior, the BMP180 published example, checksum rejection, and simulated torn FRAM writes without changing earlier slots. This is not physical sensor, power-rail, or flash hardware validation.

Primary references:

- Raspberry Pi MicroPython setup: https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
- BMI160 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf
- ADXL345 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- Bosch BMP180 datasheet, revision 2.5: compensation algorithm and UT=27898, UP=23843 worked example. Obtain the original datasheet for your module; BMP280/BME280 drivers are incompatible.
- NIST Handbook of Frequency Stability Analysis, SP 1065: https://tf.nist.gov/general/pdf/2220.pdf
- LittleFS design/limitations: https://github.com/littlefs-project/littlefs
- F-RAM technology and retention: https://www.infineon.com/products/memories/f-ram-ferroelectric-ram (the included driver targets the MB85RC256V addressing, not every F-RAM product).

