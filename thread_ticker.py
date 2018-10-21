from gevent_ticker import Ticker
from threading import Thread

import Queue


class TickerThread(Thread):
    def __init__(self, q=Queue.Queue(maxsize=0), period=1):
        Thread.__init__(self)

        self.__Q = q
        self.__period = period

    def run(self):
        clock = Ticker(period=self.__period, times=1)
        clock.start()
        while True:
            clock.next_tick()
            self.__Q.put(0)

    def get_q(self):
        return self.__Q
