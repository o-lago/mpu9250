import numpy as nu
from datetime import datetime as dtime


class MPUAvgData:
    def __init__(self, m1=nu.int32(0), m2=nu.int32(0), m3=nu.int32(0), m4=nu.int32(0),
                 avg1=nu.float64(0), avg2=nu.float64(0), avg3=nu.float64(0),
                 ava1=nu.float64(0), ava2=nu.float64(0), ava3=nu.float64(0),
                 avtmp=nu.float64(0), avm1=nu.int32(0), avm2=nu.int32(0), avm3=nu.int32(0),
                 n=0, nm=0, t=dtime.now(), tm=dtime.now(), t0=dtime.now(), t0m=dtime.now()):
        self.__m1 = m1
        self.__m2 = m2
        self.__m3 = m3
        self.__m4 = m4
        self.__avg1 = avg1
        self.__avg2 = avg2
        self.__avg3 = avg3
        self.__ava1 = ava1
        self.__ava2 = ava2
        self.__ava3 = ava3
        self.__avtmp = avtmp
        self.__avm1 = avm1
        self.__avm2 = avm2
        self.__avm3 = avm3
        self.__n = n
        self.__nm = nm
        self.__t = t
        self.__tm = tm
        self.__t0 = t0
        self.__t0m = t0m

    def set_m1(self, m1):
        self.__m1 = m1

    def get_m1(self):
        return self.__m1

    def set_m2(self, m2):
        self.__m2 = m2

    def get_m2(self):
        return self.__m2

    def set_m3(self, m3):
        self.__m3 = m3

    def get_m3(self):
        return self.__m3

    def set_m4(self, m4):
        self.__m4 = m4

    def get_m4(self):
        return self.__m4

    def set_avg1(self, avg1):
        self.__avg1 = avg1

    def get_avg1(self):
        return self.__avg1

    def set_avg2(self, avg2):
        self.__avg2 = avg2

    def get_avg2(self):
        return self.__avg2

    def set_avg3(self, avg3):
        self.__avg3 = avg3

    def get_avg3(self):
        return self.__avg3

    def set_ava1(self, ava1):
        self.__ava1 = ava1

    def get_ava1(self):
        return self.__ava1

    def set_ava2(self, ava2):
        self.__ava2 = ava2

    def get_ava2(self):
        return self.__ava2

    def set_ava3(self, ava3):
        self.__ava3 = ava3

    def get_ava3(self):
        return self.__ava3

    def set_avtmp(self, avtmp):
        self.__avtmp = avtmp

    def get_avtmp(self):
        return self.__avtmp

    def set_avm1(self, avm1):
        self.__avm1 = avm1

    def get_avm1(self):
        return self.__avm1

    def set_avm2(self, avm2):
        self.__avm2 = avm2

    def get_avm2(self):
        return self.__avm2

    def set_avm3(self, avm3):
        self.__avm3 = avm3

    def get_avm3(self):
        return self.__avm3

    def set_n(self, n):
        self.__n = n

    def get_n(self):
        return self.__n

    def set_nm(self, nm):
        self.__nm = nm

    def get_nm(self):
        return self.__nm

    def set_t(self, t):
        self.__t = t

    def get_t(self):
        return self.__t

    def set_tm(self, tm):
        self.__tm = tm

    def get_tm(self):
        return self.__tm

    def set_t0(self, t0):
        self.__t0 = t0

    def get_t0(self):
        return self.__t0

    def set_t0m(self, t0m):
        self.__t0m = t0m

    def get_t0m(self):
        return self.__t0m

    def add_avg1(self, avg1):
        self.__avg1 += nu.float64(avg1)

    def add_avg2(self, avg2):
        self.__avg2 += nu.float64(avg2)

    def add_avg3(self, avg3):
        self.__avg3 += nu.float64(avg3)

    def add_ava1(self, ava1):
        self.__ava1 += nu.float64(ava1)

    def add_ava2(self, ava2):
        self.__ava2 += nu.float64(ava2)

    def add_ava3(self, ava3):
        self.__ava3 += nu.float64(ava3)

    def add_avtmp(self, avtmp):
        self.__avtmp += nu.float64(avtmp)

    def add_avm1(self, avm1):
        self.__avm1 += nu.int32(avm1)

    def add_avm2(self, avm2):
        self.__avm2 += nu.int32(avm2)

    def add_avm3(self, avm3):
        self.__avm3 += nu.int32(avm3)

    def add_n(self, n):
        self.__n += n

    def add_nm(self, nm):
        self.__nm += nm


class MPUData:
    __DATABASE_STR_FORMAT = "%Y-%m-%d %H:%M:%S"

    def __init__(self, g1=nu.float64(0.0), g2=nu.float64(0.0), g3=nu.float64(0.0),
                 a1=nu.float64(0.0), a2=nu.float64(0.0), a3=nu.float64(0.0),
                 m1=nu.float64(0.0), m2=nu.float64(0.0), m3=nu.float64(0.0),
                 temp=nu.float64(0.0), n=1, nm=1, t=dtime.now(), tm=dtime.now(), dt=0, dtm=0,
                 msg_error=None):
        self.G1 = g1
        self.G2 = g2
        self.G3 = g3
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.M1 = m1
        self.M2 = m2
        self.M3 = m3
        self.Temp = temp
        self.T = t
        self.TM = tm
        self.DT = dt
        self.DTM = dtm
        self.N = n
        self.NM = nm
        self.MsgError = msg_error

    def get_json(self):
        return {
            "G1": self.G1,
            "G2": self.G2,
            "G3": self.G3,
            "A1": self.A1,
            "A2": self.A2,
            "A3": self.A3,
            "M1": self.M1,
            "M2": self.M2,
            "M3": self.M3,
            "Temp": self.Temp,
            "T": self.T.strftime(self.__DATABASE_STR_FORMAT),
            "TM": self.TM.strftime(self.__DATABASE_STR_FORMAT),
            "DT": self.DT,
            "DTM": self.DTM,
            "N": self.N,
            "NM": self.NM,
            "MsgError": self.MsgError
        }


class MPUCalData:
    def __init__(self,
                 g01=nu.float64(0.0), g02=nu.float64(0.0), g03=nu.float64(0.0),  # Accelerometer hardware bias
                 a01=nu.float64(0.0), a02=nu.float64(0.0), a03=nu.float64(0.0),  # Gyro hardware bias
                 m01=nu.float64(0.0), m02=nu.float64(0.0), m03=nu.float64(0.0),  # Magnetometer hardware bias
                 ms11=nu.float64(1.0), ms12=nu.float64(0.0), ms13=nu.float64(0.0),  # Magnetometer rescaling matrix
                 ms21=nu.float64(0.0), ms22=nu.float64(1.0), ms23=nu.float64(0.0),  # (Only diagonal is used currently)
                 ms31=nu.float64(0.0), ms32=nu.float64(0.0), ms33=nu.float64(1.0)):
        self.G01 = g01
        self.G02 = g02
        self.G03 = g03
        self.A01 = a01
        self.A02 = a02
        self.A03 = a03
        self.M01 = m01
        self.M02 = m02
        self.M03 = m03
        self.Ms11 = ms11
        self.Ms12 = ms12
        self.Ms13 = ms13
        self.Ms21 = ms21
        self.Ms22 = ms22
        self.Ms23 = ms23
        self.Ms31 = ms31
        self.Ms32 = ms32
        self.Ms33 = ms33

    def get_json(self):
        return {
            "G01": self.G01,
            "G02": self.G02,
            "G03": self.G03,
            "A01": self.A01,
            "A02": self.A02,
            "A03": self.A03,
            "M01": self.M01,
            "M02": self.M02,
            "M03": self.M03,
            "Ms11": self.Ms11,
            "Ms12": self.Ms12,
            "Ms13": self.Ms13,
            "Ms21": self.Ms21,
            "Ms22": self.Ms22,
            "Ms23": self.Ms23,
            "Ms31": self.Ms31,
            "Ms32": self.Ms32,
            "Ms33": self.Ms33
        }
