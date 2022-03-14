from digilent import *
import time

ad2 = AnalogDiscovery2()

while True:
    ad2.io.D0.value = not ad2.io.D0.value
    time.sleep(2)