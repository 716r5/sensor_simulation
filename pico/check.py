from sensors import bus, Sensors
import time

i = bus()
print('I2C devices:', [hex(a) for a in i.scan()])
s = Sensors(i)
print('All three chip IDs and BMI160 power/config checks passed.')
print('Columns: gyro raw xyz, BMI accel raw xyz, ADXL raw xyz, UT, UP, C, Pa')
for _ in range(30):
    print(s.read())
    time.sleep_ms(100)
print('Now check physical response using README steps. ID checks alone are not a self-test.')
