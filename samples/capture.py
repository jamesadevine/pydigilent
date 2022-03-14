from pydigilent import *
import time

ad2 = AnalogDiscovery2()
ad2.io.D0.value = 1
ad2.io.D8.value = 1

ad2.scope.channel1.vertical_division = .2
ad2.scope.horizontal_division = .0001
ad2.scope.channel1.offset = 3.
ad2.scope.channel2.offset = 2.5

ad2.scope.acquire()

while not ad2.scope.acquire_complete():
    time.sleep(.1)

ad2.plot(ad2.scope.data, show=True)
