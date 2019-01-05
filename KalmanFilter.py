import math
import struct


class KalmanFilter:
    __SAMPLE_FREQ = float(100.0)  # Sample frequency in Hz
    __BETA_DEF = float(0.020)  # 2 * proportional gain

    def __init__(self, beta=__BETA_DEF, sample_freq=__SAMPLE_FREQ,
                 q0=float(1.0), q1=float(0.0), q2=float(0.0), q3=float(0.0)):
        self.__beta = beta
        self.__sample_freq = sample_freq
        self.__q0 = q0
        self.__q1 = q1
        self.__q2 = q2
        self.__q3 = q3

    def get_q0(self):
        return self.__q0

    def get_q1(self):
        return self.__q1

    def get_q2(self):
        return self.__q2

    def get_q3(self):
        return self.__q3

    def __inv_sqrt(self, number):
        threehalfs = 1.5
        x2 = number * 0.5
        y = number

        packed_y = struct.pack('f', y)
        i = struct.unpack('i', packed_y)[0]  # Treat float's bytes as int
        i = 0x5f3759df - (i >> 1)  # Arithmetic with magic number
        packed_i = struct.pack('i', i)
        y = struct.unpack('f', packed_i)[0]  # Treat int's bytes as float

        y = y * (threehalfs - (x2 * y * y))  # Newton's method

        return y

    def filter(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """
        recip_norm = float(0.0)
        s0 = float(0.0)
        s1 = float(0.0)
        s2 = float(0.0)
        s2 = float(0.0)

        qDot1 = float(0.0)
        qDot2 = float(0.0)
        qDot3 = float(0.0)
        qDot4 = float(0.0)

        hx = float(0.0)
        hy = float(0.0)

        _2q0mx = float(0.0)
        _2q0my = float(0.0)
        _2q0mz = float(0.0)
        _2q1mx = float(0.0)
        _2bx = float(0.0)
        _2bz = float(0.0)
        _4bx = float(0.0)
        _4bz = float(0.0)
        _2q0 = float(0.0)
        _2q1 = float(0.0)
        _2q2 = float(0.0)
        _2q3 = float(0.0)
        _2q0q2 = float(0.0)
        _2q2q3 = float(0.0)
        q0q0 = float(0.0)
        q0q1 = float(0.0)
        q0q2 = float(0.0)
        q0q3 = float(0.0)
        q1q1 = float(0.0)
        q1q2 = float(0.0)
        q1q3 = float(0.0)
        q2q2 = float(0.0)
        q2q3 = float(0.0)
        q3q3 = float(0.0)
        """

        # Use IMU algorithm if magnetometer measurement invalid (avoids `None` in magnetometer normalisation)
        if mx == float(0.0) and my == float(0.0) and mz == float(0.0):
            return

        # Rate of change of quaternion from gyroscope
        qDot1 = float(0.5) * (-self.__q1 * gx - self.__q2 * gy - self.__q3 * gz)
        qDot2 = float(0.5) * (self.__q0 * gx + self.__q2 * gz - self.__q3 * gy)
        qDot3 = float(0.5) * (self.__q0 * gy - self.__q1 * gz + self.__q3 * gx)
        qDot4 = float(0.5) * (self.__q0 * gz + self.__q1 * gy - self.__q2 * gx)

        if not (ax == float(0.0) and ay == float(0.0) and az == float(0.0)):
            # Normalise accelerometer measurement
            recip_norm = self.__inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            # Normalise magnetometer measurement
            recip_norm = self.__inv_sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

            # Auxiliary variables to avoid repeated arithmetic
            _2q0mx = float(2.0) * self.__q0 * mx
            _2q0my = float(2.0) * self.__q0 * my
            _2q0mz = float(2.0) * self.__q0 * mz
            _2q1mx = float(2.0) * self.__q1 * mx
            _2q0 = float(2.0) * self.__q0
            _2q1 = float(2.0) * self.__q1
            _2q2 = float(2.0) * self.__q2
            _2q3 = float(2.0) * self.__q3
            _2q0q2 = float(2.0) * self.__q0 * self.__q2
            _2q2q3 = float(2.0) * self.__q2 * self.__q3

            q0q0 = self.__q0 * self.__q0
            q0q1 = self.__q0 * self.__q1
            q0q2 = self.__q0 * self.__q2
            q0q3 = self.__q0 * self.__q3
            q1q1 = self.__q1 * self.__q1
            q1q2 = self.__q1 * self.__q2
            q1q3 = self.__q1 * self.__q3
            q2q2 = self.__q2 * self.__q2
            q2q3 = self.__q2 * self.__q3
            q3q3 = self.__q3 * self.__q3

            # Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * self.__q3 + _2q0mz * self.__q2 + mx * q1q1 + _2q1 * my * self.__q2 + _2q1 * mz * self.__q3 - mx * q2q2 - mx * q3q3
            hy = _2q0mx * self.__q3 + my * q0q0 - _2q0mz * self.__q1 + _2q1mx * self.__q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.__q3 - my * q3q3
            _2bx = math.sqrt(hx * hx + hy * hy)
            _2bz = -_2q0mx * self.__q2 + _2q0my * self.__q1 + mz * q0q0 + _2q1mx * self.__q3 - mz * q1q1 + _2q2 * my * self.__q3 - mz * q2q2 + mz * q3q3
            _4bx = float(2.0) * _2bx
            _4bz = float(2.0) * _2bz

            # Gradient decent algorithm corrective step
            s0 = -_2q2 * (float(2.0) * q1q3 - _2q0q2 - ax) + _2q1 * (
                    float(2.0) * q0q1 + _2q2q3 - ay) - _2bz * self.__q2 * (
                         _2bx * (float(0.5) - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                         -_2bx * self.__q3 + _2bz * self.__q1) * (
                         _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.__q2 * (
                         _2bx * (q0q2 + q1q3) + _2bz * (float(0.5) - q1q1 - q2q2) - mz)
            s1 = _2q3 * (float(2.0) * q1q3 - _2q0q2 - ax) + _2q0 * (float(2.0) * q0q1 + _2q2q3 - ay) - float(
                4.0) * self.__q1 * (1 - float(2.0) * q1q1 - float(2.0) * q2q2 - az) + _2bz * self.__q3 * (
                         _2bx * (float(0.5) - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                         _2bx * self.__q2 + _2bz * self.__q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (
                         _2bx * self.__q3 - _4bz * self.__q1) * (
                         _2bx * (q0q2 + q1q3) + _2bz * (float(0.5) - q1q1 - q2q2) - mz)
            s2 = -_2q0 * (float(2.0) * q1q3 - _2q0q2 - ax) + _2q3 * (float(2.0) * q0q1 + _2q2q3 - ay) - float(
                4.0) * self.__q2 * (1 - float(2.0) * q1q1 - float(2.0) * q2q2 - az) + (
                         -_4bx * self.__q2 - _2bz * self.__q0) * (
                         _2bx * (float(0.5) - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                         _2bx * self.__q1 + _2bz * self.__q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (
                         _2bx * self.__q0 - _4bz * self.__q2) * (
                         _2bx * (q0q2 + q1q3) + _2bz * (float(0.5) - q1q1 - q2q2) - mz)
            s3 = _2q1 * (float(2.0) * q1q3 - _2q0q2 - ax) + _2q2 * (float(2.0) * q0q1 + _2q2q3 - ay) + (
                    -_4bx * self.__q3 + _2bz * self.__q1) * (
                         _2bx * (float(0.5) - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                         -_2bx * self.__q0 + _2bz * self.__q2) * (
                         _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.__q1 * (
                         _2bx * (q0q2 + q1q3) + _2bz * (float(0.5) - q1q1 - q2q2) - mz)

            recip_norm = self.__inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  # normalise step magnitude
            s0 *= recip_norm
            s1 *= recip_norm
            s2 *= recip_norm
            s3 *= recip_norm

            # Apply feedback step
            qDot1 -= self.__beta * s0
            qDot2 -= self.__beta * s1
            qDot3 -= self.__beta * s2
            qDot4 -= self.__beta * s3

        # Integrate rate of change of quaternion to yield quaternion
        self.__q0 += qDot1 * (float(1.0) / self.__sample_freq)
        self.__q1 += qDot2 * (float(1.0) / self.__sample_freq)
        self.__q2 += qDot3 * (float(1.0) / self.__sample_freq)
        self.__q3 += qDot4 * (float(1.0) / self.__sample_freq)

        # Normalise quaternion
        recip_norm = self.__inv_sqrt(
            self.__q0 * self.__q0 + self.__q1 * self.__q1 + self.__q2 * self.__q2 + self.__q3 * self.__q3)
        self.__q0 *= recip_norm
        self.__q1 *= recip_norm
        self.__q2 *= recip_norm
        self.__q3 *= recip_norm

        """
        R11 = float(2.0) * self.__q0 * self.__q0 - 1 + float(2.0) * self.__q1 * self.__q1
        R21 = float(2.0) * (self.__q1 * self.__q2 - self.__q0 * self.__q3)
        R31 = float(2.0) * (self.__q1 * self.__q3 + self.__q0 * self.__q2)
        R32 = float(2.0) * (self.__q2 * self.__q3 - self.__q0 * self.__q1)
        R33 = float(2.0) * self.__q0 * self.__q0 - 1 + float(2.0) * self.__q3 * self.__q3

        phi = math.atan2(R32, R33)
        theta = -math.atan(R31 / math.sqrt(1 - R31 * R31))
        psi = math.atan2(R21, R11)

        return {
            'phi': phi * float(180) / math.pi,
            'theta': theta * float(180) / math.pi,
            'psi': psi * float(180) / math.pi
        }
        """
