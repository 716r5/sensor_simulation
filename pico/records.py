"""Portable binary framing, CRC, and deliberately non-overwriting storage."""
import struct
import os

FMT = '<IIQ11i2f'
SIZE = struct.calcsize(FMT)  # 68 byte payload + 4 byte CRC
RECORD = SIZE + 4
SLOT = 80
HEADER = 'boot,seq,t_us,gx_raw,gy_raw,gz_raw,bx_raw,by_raw,bz_raw,ax_raw,ay_raw,az_raw,ut,up,temp_c,pressure_pa'

def crc32(data):
    c = 0xFFFFFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if c & 1 else 0)
    return c ^ 0xFFFFFFFF

def pack(boot, seq, t_us, values):
    body = struct.pack(FMT, boot, seq, t_us, *values)
    return body + struct.pack('<I', crc32(body))

def unpack(data):
    if len(data) != RECORD or crc32(data[:SIZE]) != struct.unpack('<I', data[SIZE:])[0]:
        raise ValueError('Bad length/CRC')
    return struct.unpack(FMT, data[:SIZE])

def csv(data):
    return ','.join(str(x) for x in unpack(data))

class FlashLog:
    # Prototype only. Relies on installed filesystem and flash power-loss behavior.
    def __init__(self):
        names = os.listdir()
        number = 1
        while 'log%06d.bin' % number in names:
            number += 1
        self.name = 'log%06d.bin' % number
        self.f = open(self.name, 'wb')  # guaranteed new name; never truncates an old log
        self.f.flush()
        os.sync()

    def append(self, record):
        st = os.statvfs('/')
        if st[0] * st[3] < 65536:
            raise RuntimeError('Flash nearly full: stopped without deleting old logs')
        self.f.write(record)
        self.f.flush()
        os.sync()

class FramLog:
    # MB85RC256V / compatible 32 KiB, 16-bit I2C memory addressing only.
    # Requires intentional initialization first. No circular buffer and no auto-erase.
    def __init__(self, i2c, address=0x50):
        self.i, self.address = i2c, address
        self.next = None
        self.used = 0
        for slot in range(32768 // SLOT):
            block = self.read(slot)
            if block == b'\xff' * SLOT:
                if self.next is None:
                    self.next = slot
            else:
                self.used += 1
        if self.next is None:
            raise RuntimeError('FRAM full or uninitialized: export, do not erase automatically')

    def read(self, slot):
        return self.i.readfrom_mem(self.address, slot * SLOT, SLOT, addrsize=16)

    def append(self, record):
        if self.next is None:
            raise RuntimeError('FRAM full: stopped, old records preserved')
        slot = self.next
        # A partly written slot is permanently consumed until deliberate reinitialization.
        self.i.writeto_mem(self.address, slot * SLOT, record, addrsize=16)
        if self.read(slot)[:RECORD] != record:
            raise RuntimeError('FRAM payload readback failed')
        self.i.writeto_mem(self.address, slot * SLOT + RECORD, b'\xa5', addrsize=16)
        block = self.read(slot)
        if block[RECORD] != 0xA5:
            raise RuntimeError('FRAM commit readback failed')
        unpack(block[:RECORD])
        self.next = None
        for candidate in range(slot + 1, 32768 // SLOT):
            if self.read(candidate) == b'\xff' * SLOT:
                self.next = candidate
                break

def initialize_fram(i2c, confirmation, address=0x50):
    if confirmation != 'ERASE ALL FRAM DATA':
        raise ValueError('Explicit erase confirmation required')
    for slot in range(32768 // SLOT):
        i2c.writeto_mem(address, slot * SLOT, b'\xff' * SLOT, addrsize=16)
    print('FRAM initialized. Never do this to recover data.')
