#!/usr/bin/python

from mpu9250 import MPU9250
import web
from KalmanFilter import KalmanFilter
import math

urls = (
    '/', 'index'
)

_RATIO_ACC = float(4.0) / float(32767.0)
_RATIO_GYRO = (float(1000.0) / float(32767.0)) * (math.pi / float(180.0))
# define         RATIO_GYRO      (1000./32767.)
_RATIO_MAG = float(48.0) / float(32767.0)

mpu = MPU9250()
mpu.initialize()


class index:

    def GET(self):
        avg = mpu.mpuDate

        gx = avg.G1  # (avg.G1 - float(-48.4827)) * _RATIO_GYRO
        gy = avg.G2  # (avg.G2 - float(+76.3552)) * _RATIO_GYRO
        gz = avg.G3  # (avg.G3 - float(+64.3234)) * _RATIO_GYRO
        ax = avg.A1  # avg.A1 * _RATIO_ACC
        ay = avg.A2  # avg.A2 * _RATIO_ACC
        az = avg.A3  # avg.A3 * _RATIO_ACC
        mx = avg.M1  # avg.M1 * _RATIO_MAG
        my = avg.M2  # avg.M2 * _RATIO_MAG
        mz = avg.M3  # avg.M3 * _RATIO_MAG

        kf = KalmanFilter()

        kf.filter(gx=gx, gy=gy, gz=gz, ax=ax, ay=ay, az=az, mx=mx, my=my, mz=mz)

        R11 = float(2.0) * kf.get_q0() * kf.get_q0() - 1 + float(2.0) * kf.get_q1() * kf.get_q1()
        R21 = float(2.0) * (kf.get_q1() * kf.get_q2() - kf.get_q0() * kf.get_q3())
        R31 = float(2.0) * (kf.get_q1() * kf.get_q3() + kf.get_q0() * kf.get_q2())
        R32 = float(2.0) * (kf.get_q2() * kf.get_q3() - kf.get_q0() * kf.get_q1())
        R33 = float(2.0) * kf.get_q0() * kf.get_q0() - 1 + float(2.0) * kf.get_q3() * kf.get_q3()

        phi = math.atan2(R32, R33)
        theta = -math.atan(R31 / math.sqrt(1 - R31 * R31))
        psi = math.atan2(R21, R11)

        """
              return {
                  'phi': phi * float(180) / math.pi,
                  'theta': theta * float(180) / math.pi,
                  'psi': psi * float(180) / math.pi
              }
        """

        return str(phi * float(180) / math.pi) + ' ' + str(theta * float(180) / math.pi)


if __name__ == "__main__":
    app = web.application(urls, globals())
    app.run()
