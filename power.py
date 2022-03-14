from pydigilent import *
import time

ad2 = AnalogDiscovery2()
v = 3.5
ad2.power.vplus.enable = 1
ad2.power.vplus.voltage = v

# after configuring power options, the master must be switched to enable
ad2.power.master.enable = 1 

ad2.scope.channel1.vertical_division = 1.

while ad2.scope.channel1.voltage < v:
    print(ad2.scope.channel1.voltage)
    time.sleep(.5)

print(ad2.scope.channel1.voltage)