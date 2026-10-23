"""Run with normal Python on the PC. See README for commands."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import time
import zlib

import numpy as np

FMT = '<IIQ11i2f'
SIZE = struct.calcsize(FMT)
RECORD = SIZE + 4
HEADER = 'boot,seq,t_us,gx_raw,gy_raw,gz_raw,bx_raw,by_raw,bz_raw,ax_raw,ay_raw,az_raw,ut,up,temp_c,pressure_pa'.split(',')

def decode(record):
    if len(record) != RECORD or zlib.crc32(record[:SIZE]) != struct.unpack('<I', record[SIZE:])[0]:
        raise ValueError('record CRC/length mismatch')
    return struct.unpack(FMT, record[:SIZE])

def sync(file):
    file.flush()
    os.fsync(file.fileno())

def capture(args):
    import serial
    root = Path(args.out)
    root.mkdir(parents=True, exist_ok=True)
    stem = 'capture_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())
    # Exclusive create: previous captures cannot be truncated.
    with open(root / (stem + '.wire'), 'xb') as wire, serial.Serial(args.port, 115200, timeout=1) as port:
        outputs = {}
        started = time.monotonic()
        count = 0
        print('Capturing. Close Thonny first. Ctrl+C stops; existing records remain.')
        print('Waiting for DATA lines. If necessary press Ctrl+D in Thonny before closing it.')
        pending = bytearray()
        if args.start or args.export:
            port.write(b'\x03\x03')
            time.sleep(.3)
            if args.export:
                port.write(('import export; export.' + args.export + '()\r\n').encode())
            else:
                port.write(b'\x04')  # soft reboot runs main.py after capture is listening
        try:
            while not args.seconds or time.monotonic() - started < args.seconds:
                chunk = port.read(port.in_waiting or 1)
                if not chunk:
                    continue
                wire.write(chunk)
                sync(wire)  # preserve received bytes before attempting parsing
                pending.extend(chunk)
                while b'\n' in pending:
                    line, _, rest = pending.partition(b'\n')
                    pending = bytearray(rest)
                    if not line.startswith(b'DATA,'):
                        print(line.decode(errors='replace'))
                        continue
                    try:
                        row = decode(bytes.fromhex(line[5:].decode().strip()))
                    except (ValueError, UnicodeError) as e:
                        print('REJECTED:', e, '(wire bytes retained)')
                        continue
                    boot = row[0]
                    if boot not in outputs:
                        path = root / (stem + '_boot' + str(boot) + '.csv')
                        f = open(path, 'x', newline='')
                        writer = csv.writer(f)
                        writer.writerow(HEADER)
                        sync(f)
                        outputs[boot] = (f, writer)
                        print('Saving', path)
                    f, writer = outputs[boot]
                    writer.writerow(row)
                    sync(f)
                    count += 1
                    if count % 100 == 0:
                        print(count, 'CRC-verified rows saved')
        except KeyboardInterrupt:
            pass
        finally:
            for f, _ in outputs.values():
                f.close()
        print(count, 'rows saved. Wire file:', wire.name)

def convert(args):
    # Recover raw Pico .bin files copied through Thonny, or saved USB .wire files.
    source = Path(args.input)
    if source.suffix == '.wire':
        candidates = []
        for line in source.read_bytes().splitlines():
            if line.startswith(b'DATA,'):
                try:
                    candidates.append(bytes.fromhex(line[5:].decode().strip()))
                except (ValueError, UnicodeError):
                    print('Rejected malformed wire line')
    else:
        b = source.read_bytes()
        candidates = [b[i:i+RECORD] for i in range(0, len(b), RECORD)]
    with open(args.out, 'x', newline='') as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        good = bad = 0
        for record in candidates:
            try:
                row = decode(record)
            except ValueError:
                bad += 1
                continue
            w.writerow(row)
            good += 1
        sync(f)
    print('Accepted', good, 'rejected', bad, '->', args.out)

def compare(args):
    def keyed(path):
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        result = {}
        for row in rows:
            key = (row['boot'], row['seq'])
            if key in result:
                raise ValueError('Duplicate boot/sequence in ' + path)
            result[key] = tuple(float(row[h]) for h in HEADER)
        return result
    before, after = keyed(args.before), keyed(args.after)
    if not before:
        raise ValueError('No pre-cut records to compare')
    missing = [list(k) for k in before if k not in after]
    changed = [list(k) for k in before if k in after and before[k] != after[k]]
    report = {'before_records': len(before), 'after_records': len(after),
              'missing': missing, 'changed': changed,
              'all_pre_cut_rows_recovered_exactly': not missing and not changed,
              'input_sha256': {p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
                               for p in [args.before, args.after]}}
    with open(args.out, 'x') as f:
        json.dump(report, f, indent=2)
        sync(f)
    print(json.dumps(report, indent=2))
    if missing or changed:
        raise SystemExit(1)

def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('Empty capture: ' + str(path))
    if len({r['boot'] for r in rows}) != 1:
        raise ValueError('Use one boot/session per analysis file')
    return {key: np.array([float(r[key]) for r in rows]) for key in HEADER}

def vectors(d, prefix, scale):
    return np.column_stack([d[prefix + axis + '_raw'] for axis in 'xyz']) * scale

def physical(d):
    return vectors(d, 'b', 1/16384), vectors(d, 'a', .0039), vectors(d, 'g', 1/65.6)

def six_face(paths, sensor):
    means = []
    for k, path in enumerate(paths):
        d = load(path)
        values = physical(d)[0 if sensor == 'bmi' else 1]
        if len(values) < 50 or np.max(values.std(axis=0)) > .03:
            raise ValueError('Need >=50 stable samples per face: ' + path)
        mean = values.mean(axis=0)
        expected = np.zeros(3)
        expected[k//2] = 1 if k % 2 == 0 else -1
        if np.linalg.norm(mean - expected) > .3:
            raise ValueError('Wrong orientation/axis mapping for ' + sensor + ': ' + path)
        means.append(mean)
    bias = np.array([(means[2*k][k] + means[2*k+1][k])/2 for k in range(3)])
    scale = np.array([(means[2*k][k] - means[2*k+1][k])/2 for k in range(3)])
    if np.any((scale < .7) | (scale > 1.3)):
        raise ValueError('Implausible accelerometer scale')
    return {'bias_g': bias.tolist(), 'scale': scale.tolist(), 'method': 'six-face diagonal'}

def calibrate(args):
    still = load(args.still)
    b, a, g = physical(still)
    if len(g) < 300 or np.max(g.std(axis=0)) > 1 or np.max(abs(g.mean(axis=0))) > 10:
        raise ValueError('Need >=300 stationary samples, no rotation, plausible gyro bias')
    if max(np.max(b.std(axis=0)), np.max(a.std(axis=0))) > .03:
        raise ValueError('Stationary recording moved')
    result = {
        'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'bmi': six_face(args.faces, 'bmi'),
        'adxl': six_face(args.adxl_faces or args.faces, 'adxl'),
        'gyro_bias_dps': g.mean(axis=0).tolist(),
        'gyro_stationary_std_dps': g.std(axis=0).tolist(),
        'p0_pa': float(still['pressure_pa'].mean()),
        'pressure_offset_pa': 0.0,
        'pressure_absolute_calibrated': args.pressure_reference_pa is not None,
        'inputs_sha256': {p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
                          for p in set([args.still] + args.faces + (args.adxl_faces or []))},
    }
    if args.pressure_reference_pa is not None:
        result['pressure_offset_pa'] = args.pressure_reference_pa - result['p0_pa']
        result['p0_pa'] = args.pressure_reference_pa
    with open(args.out, 'x') as f:
        json.dump(result, f, indent=2)
        sync(f)
    print('Calibration saved:', args.out)
    print('Gyro bias dps:', result['gyro_bias_dps'])
    print('Pressure uses factory compensation; external absolute reference:', result['pressure_absolute_calibrated'])

class TiltEKF:
    """Two-state nonlinear roll/pitch EKF. Inputs g and rad/s. Bias fixed by calibration."""
    def __init__(self, accel):
        self.x = np.array([np.arctan2(accel[1], accel[2]),
                           np.arctan2(-accel[0], np.hypot(accel[1], accel[2]))])
        self.P = np.eye(2) * np.deg2rad(5)**2

    def step(self, gyro, accel, dt):
        phi, theta = self.x
        if abs(theta) > np.deg2rad(75):
            raise ValueError('Tilt EKF near Euler singularity. Use a quaternion EKF for large rotations.')
        p, q, r = gyro
        s, c, t = np.sin(phi), np.cos(phi), np.tan(theta)
        sec = 1/np.cos(theta)
        F = np.array([[1 + dt*(c*q-s*r)*t, dt*(s*q+c*r)*sec**2],
                      [dt*(-s*q-c*r), 1]])
        G = np.array([[1, s*t, c*t], [0, c, -s]])
        self.x += dt*np.array([p+(s*q+c*r)*t, c*q-s*r])
        self.P = F @ self.P @ F.T + (np.deg2rad(.5)*dt)**2 * G @ G.T + np.eye(2)*1e-8*dt
        used = False
        # Norm gate is a basic check; constant sustained acceleration may still fool tilt.
        if .85 < np.linalg.norm(accel) < 1.15:
            phi, theta = self.x
            s, c, st, ct = np.sin(phi), np.cos(phi), np.sin(theta), np.cos(theta)
            h = np.array([-st, s*ct, c*ct])
            H = np.array([[0, -ct], [c*ct, -s*st], [-s*ct, -c*st]])
            R = np.eye(3) * .03**2  # starter tuning in g^2, not characterized flight noise
            residual = accel - h
            S = H @ self.P @ H.T + R
            if residual @ np.linalg.solve(S, residual) < 16.27:
                K = np.linalg.solve(S, H @ self.P).T
                self.x += K @ residual
                A = np.eye(2) - K @ H
                self.P = A @ self.P @ A.T + K @ R @ K.T  # Joseph form
                used = True
        self.x[0] = (self.x[0] + np.pi) % (2*np.pi) - np.pi
        self.P = (self.P + self.P.T)/2
        return np.rad2deg(self.x.copy()), used

def variances(y, dt):
    n = len(y)
    result = []
    # Keep >=10 disjoint triples at the largest tau. Overlaps are not independent trials.
    for m in np.unique(np.geomspace(1, max(1, n//30), 35).astype(int)):
        if n < 3*m:
            continue
        c = np.r_[0., np.cumsum(y - np.mean(y))]
        avg = (c[m:] - c[:-m])/m
        av = np.mean((avg[m:] - avg[:-m])**2)/2
        hv = np.mean((avg[2*m:] - 2*avg[m:-m] + avg[:-2*m])**2)/6
        result.append((m*dt, av, hv, n-2*m+1, n-3*m+1))
    return np.array(result)

def analyze(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d = load(args.input)
    cal_text = Path(args.cal).read_text()
    cal = json.loads(cal_text)
    t = (d['t_us'] - d['t_us'][0])/1e6
    if len(t) < 3 or np.any(np.diff(t) <= 0) or np.any(np.diff(d['seq']) != 1):
        raise ValueError('Need >=3 consecutive records, one boot, no duplicate/missing sequences')
    intervals = np.diff(t)
    dt = float(np.median(intervals))
    jitter = float(np.max(abs(intervals-dt))/dt)
    if np.max(intervals) > .5:
        raise ValueError('Too large a sample gap for this 10 Hz tilt demonstration')
    b, a, g = physical(d)
    b = (b - cal['bmi']['bias_g']) / cal['bmi']['scale']
    a = (a - cal['adxl']['bias_g']) / cal['adxl']['scale']
    g = g - cal['gyro_bias_dps']
    pressure = d['pressure_pa'] + cal['pressure_offset_pa']
    altitude = 44330*(1-(pressure/cal['p0_pa'])**.190294957)
    tilt = TiltEKF(b[0])
    estimates = [np.rad2deg(tilt.x.copy())]
    updates = [False]
    for k in range(1, len(t)):
        angles, used = tilt.step(np.deg2rad(g[k-1]), b[k], intervals[k-1])
        estimates.append(angles)
        updates.append(used)
    estimates = np.array(estimates)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)  # fail instead of overwriting earlier results
    names = ['bmi_' + c + '_g' for c in 'xyz'] + ['adxl_' + c + '_g' for c in 'xyz'] + ['gyro_' + c + '_dps' for c in 'xyz'] + ['pressure_pa', 'temp_c']
    signals = np.column_stack([b, a, g, pressure, d['temp_c']])
    np.savetxt(out/'calibrated.csv', np.column_stack([d['boot'], d['seq'], t, signals, altitude, estimates, updates]),
               delimiter=',', header=','.join(['boot','seq','t_s']+names+['relative_altitude_m','roll_deg','pitch_deg','accel_update_used']), comments='')
    (out/'calibration_used.json').write_text(cal_text)
    summary = {'samples': len(t), 'duration_s': float(t[-1]), 'median_dt_s': dt,
               'max_relative_interval_jitter': jitter, 'missing_sequences': 0,
               'calibration_sha256': hashlib.sha256(cal_text.encode()).hexdigest(),
               'stationary_claimed_by_user': args.stationary,
               'variance_written': False,
               'note': 'Tilt uses BMI160 only. ADXL is independent comparison; barometer reports relative altitude. No yaw/position estimate.'}
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, estimates); axes[0].legend(['EKF roll','EKF pitch']); axes[0].set_ylabel('degrees')
    axes[1].plot(t, np.linalg.norm(b, axis=1), label='BMI160')
    axes[1].plot(t, np.linalg.norm(a, axis=1), label='ADXL345'); axes[1].legend(); axes[1].set_ylabel('accel norm (g)')
    axes[2].plot(t, altitude); axes[2].set_ylabel('relative altitude (m)'); axes[2].set_xlabel('time (s)')
    fig.tight_layout(); fig.savefig(out/'sensor_and_ekf.png', dpi=160); plt.close(fig)
    if args.stationary:
        if jitter > .10:
            summary['variance_rejected_reason'] = 'Interval jitter exceeds 10%; do not interpolate/bridge gaps for this estimator'
        elif len(t) < 300:
            summary['variance_rejected_reason'] = 'Need >=300 samples; use >=30 minutes for useful stability results'
        elif np.max(g.std(axis=0)) > 1 or max(np.max(b.std(axis=0)), np.max(a.std(axis=0))) > .03:
            summary['variance_rejected_reason'] = 'Stationarity sanity check failed'
        else:
            fig, axes = plt.subplots(4, 1, figsize=(10, 13))
            with open(out/'variances.csv', 'x', newline='') as f:
                w = csv.writer(f)
                w.writerow(['channel','tau_s','allan_variance','hadamard_variance','allan_terms','hadamard_terms'])
                for j, name in enumerate(names):
                    rows = variances(signals[:, j], dt)
                    for row in rows:
                        w.writerow([name]+row.tolist())
                    panel = 0 if j < 6 else 1 if j < 9 else j-7
                    axes[panel].loglog(rows[:,0], np.sqrt(np.maximum(rows[:,1], 1e-30)), label=name+' Allan')
                    axes[panel].loglog(rows[:,0], np.sqrt(np.maximum(rows[:,2], 1e-30)), '--', label=name+' Hadamard')
            for ax, unit in zip(axes, ['g','degrees/s','Pa','degrees C']):
                ax.set_xlabel('averaging time tau (s)'); ax.set_ylabel('deviation ('+unit+')')
                ax.legend(fontsize=6); ax.grid(True, which='both', alpha=.3)
            fig.tight_layout(); fig.savefig(out/'stability_deviation.png', dpi=160); plt.close(fig)
            summary['variance_written'] = True
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print('Results:', out)

def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='command', required=True)
    s = sub.add_parser('capture'); s.add_argument('--port', required=True); s.add_argument('--out', default='logs'); s.add_argument('--seconds', type=float, default=0)
    modes = s.add_mutually_exclusive_group(); modes.add_argument('--start', action='store_true'); modes.add_argument('--export', choices=['flash','fram']); s.set_defaults(func=capture)
    s = sub.add_parser('convert'); s.add_argument('input'); s.add_argument('--out', required=True); s.set_defaults(func=convert)
    s = sub.add_parser('compare'); s.add_argument('--before', required=True); s.add_argument('--after', required=True); s.add_argument('--out', required=True); s.set_defaults(func=compare)
    s = sub.add_parser('calibrate'); s.add_argument('--still', required=True); s.add_argument('--faces', nargs=6, required=True, metavar='CSV'); s.add_argument('--adxl-faces', nargs=6); s.add_argument('--pressure-reference-pa', type=float); s.add_argument('--out', default='calibration.json'); s.set_defaults(func=calibrate)
    s = sub.add_parser('analyze'); s.add_argument('input'); s.add_argument('--cal', required=True); s.add_argument('--out', required=True); s.add_argument('--stationary', action='store_true'); s.set_defaults(func=analyze)
    args = p.parse_args()
    args.func(args)

if __name__ == '__main__':
    main()
