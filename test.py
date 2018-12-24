#!/usr/bin/python

from mpu9250 import MPU9250
import time

mpu = MPU9250()
mpu.initialize()

while True:
    time.sleep(10)
    print (mpu.get_avg().get_json())
