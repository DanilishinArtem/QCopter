# https://github.com/micropython-IMU/micropython-mpu9x50
from machine import Pin, I2C
from mpu9250 import MPU9250
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8))
imu = MPU9250(i2c)

while True:
    print(imu.accel.xyz)
    print(imu.gyro.xyz)
    print(imu.mag.xyz)
    print(imu.temperature)
    print(imu.accel.z)
    time.sleep(1)

