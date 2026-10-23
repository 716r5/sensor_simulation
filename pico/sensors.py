"""Small MicroPython drivers. Units/scales are applied on the PC, not here."""
from machine import I2C, Pin
import struct
import time

def bus():
    return I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

class Sensors:
    def __init__(self, i2c):
        self.i = i2c
        found = i2c.scan()
        self.bmi = 0x68 if 0x68 in found else 0x69
        self.adxl = 0x53 if 0x53 in found else 0x1D
        for address, reg, expected in ((self.bmi, 0, 0xD1),
                                       (self.adxl, 0, 0xE5), (0x77, 0xD0, 0x55)):
            actual = self.r(address, reg, 1)[0]
            if actual != expected:
                raise RuntimeError('Bad chip ID at %02x: %02x' % (address, actual))
        self.w(self.bmi, 0x7E, 0xB6)  # reset
        time.sleep_ms(100)
        self.w(self.bmi, 0x7E, 0x11)  # accel normal
        time.sleep_ms(10)
        self.w(self.bmi, 0x7E, 0x15)  # gyro normal
        time.sleep_ms(100)
        self.w(self.bmi, 0x40, 0x26)  # accel 25 Hz, normal filtering
        self.w(self.bmi, 0x41, 0x03)  # +/-2 g
        self.w(self.bmi, 0x42, 0x26)  # gyro 25 Hz, normal filtering
        self.w(self.bmi, 0x43, 0x02)  # +/-500 degrees/s
        self.w(self.adxl, 0x2D, 0)
        self.w(self.adxl, 0x31, 0x08) # full-resolution, +/-2 g
        self.w(self.adxl, 0x2C, 0x08) # 25 Hz
        self.w(self.adxl, 0x2D, 0x08) # measurement mode
        self.c = struct.unpack('>hhhHHHhhhhh', self.r(0x77, 0xAA, 22))
        if all(v == 0 for v in self.c) or self.c[3] == 0:
            raise RuntimeError('Invalid BMP180 calibration coefficients')
        time.sleep_ms(200)
        error = self.r(self.bmi, 2, 1)[0]
        pmu = self.r(self.bmi, 3, 1)[0]
        if error or (pmu & 0x3C) != 0x14:
            raise RuntimeError('BMI160 configuration error: %02x PMU %02x' % (error, pmu))

    def r(self, addr, reg, n):
        return self.i.readfrom_mem(addr, reg, n)

    def w(self, addr, reg, value):
        self.i.writeto_mem(addr, reg, bytes([value]))

    def read(self):
        # Sequential reads, NOT simultaneous measurements. Target logging: 10 Hz.
        if (self.r(self.bmi, 0x1B, 1)[0] & 0xC0) != 0xC0:
            raise RuntimeError('BMI160 sample not ready')
        if not (self.r(self.adxl, 0x30, 1)[0] & 0x80):
            raise RuntimeError('ADXL345 sample not ready')
        bmi = struct.unpack('<6h', self.r(self.bmi, 0x0C, 12))
        adxl = struct.unpack('<3h', self.r(self.adxl, 0x32, 6))
        self.w(0x77, 0xF4, 0x2E)
        time.sleep_ms(5)
        ut = struct.unpack('>H', self.r(0x77, 0xF6, 2))[0]
        self.w(0x77, 0xF4, 0x34)  # OSS=0, 4.5 ms pressure conversion
        time.sleep_ms(5)
        b = self.r(0x77, 0xF6, 3)
        up = ((b[0] << 16) | (b[1] << 8) | b[2]) >> 8
        temp, pressure = compensate(self.c, ut, up)
        return bmi + adxl + (ut, up, temp, pressure)

def compensate(c, ut, up):
    # Bosch integer compensation, OSS=0; signed division truncates toward zero.
    ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md = c
    x1 = ((ut - ac6) * ac5) >> 15
    numerator, denominator = mc << 11, x1 + md
    x2 = abs(numerator) // abs(denominator)
    if (numerator < 0) != (denominator < 0):
        x2 = -x2
    b5 = x1 + x2
    temp = ((b5 + 8) >> 4) / 10
    b6 = b5 - 4000
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11
    x2 = (ac2 * b6) >> 11
    b3 = ((ac1 * 4 + x1 + x2) + 2) >> 2
    x1 = (ac3 * b6) >> 13
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (ac4 * (x3 + 32768)) >> 15
    b7 = (up - b3) * 50000
    p = (b7 * 2) // b4 if b7 < 0x80000000 else (b7 // b4) * 2
    x1 = (p >> 8) * (p >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * p) >> 16
    p += (x1 + x2 + 3791) >> 4
    return temp, p
