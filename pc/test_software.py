"""Software tests only. No claim of hardware/power-rail qualification."""
from pathlib import Path
import sys
import types
import importlib.util
import numpy as np
import work

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'pico'))
import records

def require(condition, message):
    if not condition:
        raise AssertionError(message)

def expect_error(function):
    try:
        function()
    except (ValueError, RuntimeError, OSError):
        return
    raise AssertionError('Expected an error')

values = (10,-20,30,100,200,16000,5,6,250,27898,23843,15.,69964.)
r = records.pack(123, 7, 1000000, values)
require(records.RECORD == 72, 'frame size')
require(records.unpack(r) == work.decode(r), 'Pico/PC frame interoperability')
for j in range(len(r)):
    broken = bytearray(r)
    broken[j] ^= 1
    expect_error(lambda: work.decode(broken))
print('PASS framing and every-byte CRC corruption rejection')

fake_machine = types.ModuleType('machine')
fake_machine.I2C = object
fake_machine.Pin = object
sys.modules['machine'] = fake_machine
import sensors
coefficients = (408,-72,-14383,32741,32757,23153,6190,4,-32768,-8711,2868)
temp, pressure = sensors.compensate(coefficients, 27898, 23843)
require(temp == 15 and pressure == 69964, 'Bosch BMP180 worked example')
print('PASS BMP180 datasheet example: 15 C, 69964 Pa')

class FakeI2C:
    def __init__(self, memory=None):
        self.memory = bytearray(b'\xff'*32768 if memory is None else memory)
        self.budget = None

    def readfrom_mem(self, address, offset, count, addrsize=16):
        return bytes(self.memory[offset:offset+count])

    def writeto_mem(self, address, offset, data, addrsize=16):
        for j, byte in enumerate(data):
            if self.budget == 0:
                raise OSError('simulated loss of power')
            self.memory[offset+j] = byte
            if self.budget is not None:
                self.budget -= 1

memory = FakeI2C()
log = records.FramLog(memory)
log.append(r)
baseline = bytes(memory.memory)
second = records.pack(123, 8, 1100000, values)
for budget in range(records.RECORD + 2):
    i = FakeI2C(baseline)
    logger = records.FramLog(i)
    i.budget = budget
    try:
        logger.append(second)
    except OSError:
        pass
    require(i.memory[:records.SLOT] == baseline[:records.SLOT], 'Earlier slot changed')
    recovered = []
    for slot in range(32768//records.SLOT):
        block = i.readfrom_mem(0x50, slot*records.SLOT, records.SLOT)
        if block[records.RECORD] == 0xA5:
            recovered.append(records.unpack(block[:records.RECORD]))
    require(recovered[0] == records.unpack(r), 'Earlier committed record lost')
    i.budget = None
    reboot = records.FramLog(i)
    old = bytes(i.memory)
    reboot.append(records.pack(124, 0, 1, values))
    require(i.memory[:records.SLOT] == old[:records.SLOT], 'Reboot overwrote old record')
    if budget >= records.RECORD + 1:
        require(len(recovered) == 2, 'Completed second commit missing')
full = FakeI2C(b'\x00'*32768)
expect_error(lambda: records.FramLog(full))
require(full.memory == b'\x00'*32768, 'Full store modified')
print('PASS FRAM power interruption at each byte boundary, restart, and full-store protection')

constant = work.variances(np.ones(1000)*8, .1)
require(np.all(constant[:,1:3] == 0), 'Constant variances')
ramp = work.variances(np.arange(1000)*.01, .1)
require(np.max(ramp[:,2]) < 1e-24, 'Hadamard rejects linear trend')
require(np.allclose(ramp[:,1], (ramp[:,0]/.1*.01)**2/2), 'Allan linear trend formula')
rng = np.random.default_rng(21)
white = rng.normal(0, 2, 200000)
rows = work.variances(white, .1)
require(abs(rows[0,1]-4) < .08 and abs(rows[0,2]-4) < .08, 'White noise variance normalization')
require(abs(rows[10,1]*round(rows[10,0]/.1)-4) < .3, 'Averaging white noise scales as 1/m')
print('PASS constant, ramp, and white-noise Allan/Hadamard numerical checks')

ekf = work.TiltEKF(np.array([0.,0.,1.]))
for _ in range(1000):
    angle, used = ekf.step(np.zeros(3), np.array([0.,0.,1.]), .1)
    require(np.linalg.eigvalsh(ekf.P).min() > 0, 'Covariance positive definite')
require(np.max(abs(angle)) < 1e-10, 'Level tilt estimate')
ekf = work.TiltEKF(np.array([0.,0.,1.]))
for k in range(1,101):
    roll = np.deg2rad(k*.3)
    accel = np.array([0.,np.sin(roll),np.cos(roll)])
    angle, _ = ekf.step(np.deg2rad([3.,0.,0.]), accel, .1)
require(abs(angle[0]-30) < .1 and abs(angle[1]) < .1, 'Known roll motion')
ekf = work.TiltEKF(np.array([0.,0.,1.]))
for k in range(1,101):
    pitch = np.deg2rad(k*.3)
    accel = np.array([-np.sin(pitch),0.,np.cos(pitch)])
    angle, _ = ekf.step(np.deg2rad([0.,3.,0.]), accel, .1)
require(abs(angle[1]-30) < .1 and abs(angle[0]) < .1, 'Known pitch motion')
_, used = ekf.step(np.zeros(3), np.array([0.,0.,3.]), .1)
require(not used, 'Acceleration gate')
print('PASS level, known roll/pitch, acceleration gate, covariance checks')
print('ALL SOFTWARE CHECKS PASSED. Physical hardware is still untested.')
