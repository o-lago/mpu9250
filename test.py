#!/usr/bin/python

from mpu9250 import MPU9250
import time

mpu = MPU9250()
mpu.initialize()

i = 0
avg = mpu.get_avg()

while avg.N == 0 and i < 5:
    avg = mpu.get_avg()
    i += 1

print (avg.get_json())
