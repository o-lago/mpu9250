#!/usr/bin/python

from mpu9250 import MPU9250
import time

mpu = MPU9250()
mpu.initialize()


time.sleep(10)
avg = mpu.get_avg()
data = mpu.mpuDate
print (avg.get_json())
print '#########################'
print (data.get_json())
