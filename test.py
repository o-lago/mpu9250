#!/usr/bin/python

from mpu9250 import MPU9250
import time

mpu = MPU9250()
mpu.initialize()

time.sleep(20)
while True:
    time.sleep(1)
    print (mpu.get_avg().get_json())

