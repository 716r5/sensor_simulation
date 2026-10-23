"""Run from Thonny after Ctrl+C. Export emits the same CRC-framed serial records."""
from sensors import bus
from records import FramLog, RECORD, SLOT, unpack
import os
import binascii

def emit(record):
    unpack(record)
    print('DATA,' + binascii.hexlify(record).decode())

def flash():
    for name in sorted(os.listdir()):
        if not (name.startswith('log') and name.endswith('.bin')):
            continue
        print('# FILE', name)
        with open(name, 'rb') as f:
            while True:
                record = f.read(RECORD)
                if not record:
                    break
                try:
                    emit(record)
                except ValueError:
                    print('# INVALID record in', name, '- retained, not silently accepted')

def fram():
    i = bus()
    for slot in range(32768 // SLOT):
        block = i.readfrom_mem(0x50, slot * SLOT, SLOT, addrsize=16)
        if block == b'\xff' * SLOT:
            continue
        try:
            if block[RECORD] != 0xA5:
                raise ValueError('Uncommitted')
            emit(block[:RECORD])
        except ValueError:
            print('# INVALID/UNCOMMITTED slot', slot, '- old valid slots remain readable')
