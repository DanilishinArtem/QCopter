from machine import Pin, I2C
import mpu9250
import time

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = mpu9250.MPU9250(i2c)

while True:
    accel = mpu.acceleration
    gyro = mpu.gyro
    mag = mpu.magnetic

    print("Accelerometer:", accel)
    print("Gyroscope:", gyro)
    print("Magnetometer:", mag)

    time.sleep(1)
