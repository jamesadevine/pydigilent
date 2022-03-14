# Py Digilent

This repository contains an object oriented library for Digilent devices. It is written in Python 3 and has typing information. Typing means this library incompatible with earlier python versions.

This library has been developed and tested with the Digilent Analog Discovery 2. The library is written to support any digilent device supported by the digilent SDK. Class properties are generated at runtime based upon the connected hardware.

## Installation

Via pip:

```
pip install pydigilent
```

## Samples

### Blinkity blink

```py
from pydigilent import *
import time

ad2 = AnalogDiscovery2()

while True:
    ad2.io.D0.value = not ad2.io.D0.value
    time.sleep(2)
```

### Power control

```py
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
```

### Scope

```py
from pydigilent import *
import time

ad2 = AnalogDiscovery2()

# D0 and D8 are connected to scope channels 1 and 2 respectively
ad2.io.D0.value = 1
ad2.io.D8.value = 1

ad2.scope.channel1.vertical_division = .2
ad2.scope.horizontal_division = .0001
ad2.scope.channel1.offset = 3.
ad2.scope.channel2.offset = 2.5

ad2.scope.acquire()

while not ad2.scope.acquire_complete():
    time.sleep(.1)

# show calls plt.show (displays in a separate window)
ad2.plot(ad2.scope.data, show=True)
```

