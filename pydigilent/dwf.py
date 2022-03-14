from ctypes import *
import sys
from .dwfconstants import *
import time
import atexit
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional

dwf = None
dmgr = None
ftd = None
dftd = None

__devices = []

def __digilent_exit():
    for d in __devices:
        d.close()

_V_DIVS = 10.
_H_DIVS = 10.

atexit.register(__digilent_exit)

def dll_load():
    global dwf, dmgr, ftd, fdtd
    if sys.platform.startswith("win"):
        dwf = cdll.dwf
        dmgr = cdll.dmgr
        ftd = windll.ftd2xx
    elif sys.platform.startswith("darwin"):
        dwf = cdll.LoadLibrary("/Library/Frameworks/dwf.framework/dwf")
        dmgr = cdll.LoadLibrary(
            "/Library/Frameworks/dwf.framework/Frameworks/libdmgr.dylib")
        ftd = cdll.LoadLibrary(
            "/Library/Frameworks/dwf.framework/Frameworks/libftd2xx.dylib")
        dftd = cdll.LoadLibrary(
            "/Library/Frameworks/dwf.framework/Frameworks/libdftd2xx.dylib")
    else:
        dwf = cdll.LoadLibrary("libdwf.so")
        dmgr = cdll.LoadLibrary("libdmgr.so")
        ftd = cdll.LoadLibrary("libftd2xx.so")

    version = create_string_buffer(32)

    if sys.platform.startswith("darwin"):
        print("")
        if dftd.DFT_Init() == 0:
            print("DFT_INIT failed")
            quit()

        print("Digilent FTDI Enumeration library loaded")

        # if dftd.DFT_CreateDeviceInfoList(byref(cDev)) != 0 :
        #     print("DFT_CreateDeviceInfoList failed")
        #     dftd.DFT_Term()
        #     quit()

        # print("Devices: "+str(cDev.value))

        # for i in range(0, cDev.value):
        #     if dftd.DFT_GetDeviceInfoDetail(c_int(i), byref(flags), byref(type), byref(pdid), byref(locid), sn, name, None) != 0 :
        #         print("Failed DFT_GetDeviceInfoDetail")
        #         dftd.DFT_Term()
        #         quit()
        #     print(" "+str(i+1)+". SN:"+str(sn.value)+" '"+str(name.value)+"'"+" flags: "+hex(flags.value)+" type: "+hex(type.value)+" id: "+hex(pdid.value)+" locid: "+hex(locid.value))

    dwf.FDwfGetVersion(version)
    print("DWF Version:  %s" % str(version.value,encoding="utf8"))


class DigilentDevice:
    class DeviceType:
        EExplorer = "EExplorer"
        Discovery = "Discovery"
        Discovery2 = "Discovery2"
        DDiscovery = "DDiscovery"
        ADP3X50 = "ADP3X50"
        ADP5250 = "ADP5250"
        Any = "Any"
        
    serial = ""
    name = ""
    type = ""
    idx = -1
    __handle = None

    # Call catches any errors that may arise during execution
    # e.g. if two programs open the same device...
    def call(self, fptr, *params):
        fptr(*params)
        if self.error:
            raise Exception(self.error_message)

    def __init__(self, idx):
        name = create_string_buffer(64)
        sn = create_string_buffer(64)
        dtype = c_int()
        dwf.FDwfEnumDeviceType(idx, byref(dtype))
        dwf.FDwfEnumSN(idx, byref(sn))
        dwf.FDwfEnumDeviceName(idx, byref(name))
        self.serial = str(sn.value, encoding="utf8")
        self.name = str(name.value, encoding="utf8")
        self.idx = idx
        self._type = dtype.value

    @property
    def error(self):
        err = c_int()
        dwf.FDwfGetLastError(byref(err))
        return err.value != dwfercNoErc.value
        
    @property
    def error_message(self):
        msg = (c_byte * 512)()
        dwf.FDwfGetLastErrorMsg(msg)
        return str(msg, encoding="utf8")

    @property
    def type(self):
        if self._type == devidEExplorer.value:
            return DigilentDevice.DeviceType.EExplorer
        elif self._type == devidDiscovery.value:
            return DigilentDevice.DeviceType.Discovery
        elif self._type == devidDiscovery2.value:
            return DigilentDevice.DeviceType.Discovery2
        elif self._type == devidDDiscovery.value:
            return DigilentDevice.DeviceType.DDiscovery
        elif self._type == devidADP3X50.value:
            return DigilentDevice.DeviceType.ADP3X50
        elif self._type == devidADP5250.value:
            return DigilentDevice.DeviceType.ADP5250
        return "Unknown"

    @property
    def handle(self):
        if self.__handle is None:
            self.open()
        return self.__handle

    def open(self):
        if self.__handle is not None:
            return
        self.__handle = c_int()
        self.call(dwf.FDwfDeviceOpen, c_int(self.idx), byref(self.__handle))

    def close(self):
        if self.__handle is None or self.__handle.value == 0:
            return
        self.call(dwf.FDwfDeviceClose, self.__handle)
        self.__handle = None

    def close_all():
        global __devices
        dwf.FDwfDeviceCloseAll()

        for d in __devices:
            d.handle = None

    def __del__(self):
        self.close()

    def __str__(self):
        return "<%s[%s] \"%s\">" % (self.type, self.serial, self.name)

    def __repr__(self):
        return self.__str__()


class AnalogDiscovery2:
    def __init__(self, device: DigilentDevice = None):
        if device is None:
            global retrieve_devices
            device = retrieve_device(DigilentDevice.DeviceType.Discovery2, raw=True)

        if device is None:
            raise Exception("No Digilent device found.")
        
        self.device = device

        self.scope = DigilentScope(self.device)
        self.io = DigilentDigitalIOBus(self.device)
        self.power = DigilentPower(self.device)

    def __str__(self):
        return "<%s[%s] \"%s\">" % (self.device.type, self.device.serial, self.device.name)

    def __repr__(self):
        return self.__str__()

    def plot(self, trace, show=False):
        main = trace
        if isinstance(trace,tuple):
            main = trace[0]
        else:
            # coherce into list to avoid code duplication later
            trace = [trace]

        yvalmin = np.min(main["y"])
        ydispmax=np.max(main["yaxis"])
        if ydispmax < yvalmin:
            raise Exception("Rendering data outside plot. Adjust scope offset.")
        
        plt.xlabel(main["xlabel"])
        plt.ylabel(main["ylabel"])
        plt.xticks(main["xticks"], rotation=90)
        plt.yticks(main["yticks"])
        plt.xlim([0, np.max(main["x"])])
        plt.ylim([np.min(main["yaxis"]), np.max(main["yaxis"])])
        plt.grid()

        for t in trace:
            plt.plot(t["x"], t["y"])

        if show:
            plt.show()

class DigilentFactory:
    @staticmethod
    def coherce(device):
        if device is None:
            raise Exception("No Digilent device found.")

        if device.type == "Discovery2":
            return AnalogDiscovery2(device)
        return None

def _init_devices():
    if dwf is None:
        dll_load()

    global __devices

    count = c_int()
    dwf.FDwfEnum(enumfilterAll, byref(count))

    devs = [DigilentDevice(d) for d in range(0, count.value)]

    if __devices == []:
        __devices = devs
    else:
        for d in devs:
            if d not in devs:
                __devices += d
        for d in __devices:
            if d not in devs:
                del d

def retrieve_devices(device_filter: DigilentDevice.DeviceType, raw=False):
    _init_devices()

    devs = [d for d in __devices]

    if device_filter is not DigilentDevice.DeviceType.Any:
        devs = list(filter(lambda x: x.type == device_filter, devs))

    if raw:
        return devs

    return [DigilentFactory.coherce(d) for d in devs]

def retrieve_device(filter: DigilentDevice.DeviceType, raw=False):
    devices = retrieve_devices(filter, raw=raw)

    if len(devices) == 0:
        return None
    
    return devices[0]

class DigilentScopeTrigger:
    class TriggerCondition:
        Rising = DwfTriggerSlopeRise.value
        Falling = DwfTriggerSlopeFall.value
        Either = DwfTriggerSlopeEither.value

    class TriggerSource:
        SourceNone = trigsrcNone.value
        PC = trigsrcPC.value
        DetectorAnalogIn = trigsrcDetectorAnalogIn.value
        DetectorDigitalIn = trigsrcDetectorDigitalIn.value
        AnalogIn = trigsrcAnalogIn.value
        DigitalIn =trigsrcDigitalIn.value
        DigitalOut = trigsrcDigitalOut.value
        AnalogOut1 = trigsrcAnalogOut1.value
        AnalogOut2 = trigsrcAnalogOut2.value
        AnalogOut3 = trigsrcAnalogOut3.value
        AnalogOut4 = trigsrcAnalogOut4.value
        External1 = trigsrcExternal1.value
        External2 = trigsrcExternal2.value
        External3 = trigsrcExternal3.value
        External4 = trigsrcExternal4.value
        High = trigsrcHigh.value
        Low = trigsrcLow.value
        Clock = trigsrcClock.value

    def __init__(self, device, scope):
        self.device = device
        self.scope = scope

        self.bounds = {
            "level" :{
                "min": -1,
                "max": -1
            },
            "channel": {
                "min": -1,
                "max": -1
            }
        }

        # retrieve bounds for function calls
        self.__level_bounds()
        self.__channel_bounds()

    def __level_bounds(self):
        vmin = c_int()
        vmax = c_int()
        nsteps = c_int() # ignored
        self.device.call(dwf.FDwfAnalogInTriggerLevelInfo, self.device.handle, byref(vmin), byref(vmax), byref(nsteps))
        self.bounds["level"]["min"] = vmin.value
        self.bounds["level"]["max"] = vmax.value

    def __channel_bounds(self):
        cmin = c_int()
        cmax = c_int()
        self.device.call(dwf.FDwfAnalogInTriggerChannelInfo, self.device.handle, byref(cmin), byref(cmax))
        self.bounds["channel"]["min"] = cmin.value
        self.bounds["channel"]["max"] = cmax.value

    @property
    def level(self) -> float:
        lev = c_double()
        self.device.call(dwf.FDwfAnalogInTriggerLevelGet, self.device.handle, byref(lev))
        return lev.value

    @level.setter
    def level(self, level:float):
        if level < self.bounds["level"]["min"] or level > self.bounds['level']["max"]:
            raise Exception("Level out of bounds")
        self.device.call(dwf.FDwfAnalogInTriggerLevelSet, self.device.handle, c_double(level))

    @property 
    def source(self):
        src = c_int()
        self.device.call(dwf.FDwfAnalogInTriggerSourceGet, self.device.handle, byref(src))
        return src.value

    ###
    # Set the trigger source. 
    # @param src A class property of TriggerSource.
    ###
    @source.setter
    def source(self, src):
        if isinstance(src, DigilentScopeChannel):
            src = DigilentScopeTrigger.TriggerSource.DetectorAnalogIn
            self.channel = src.idx
        self.device.call(dwf.FDwfAnalogInTriggerSourceSet, self.device.handle, c_int(src))

    @property
    def channel(self):
        chan = c_int()
        dwf.FDwfAnalogInTriggerChannelGet(self.device.handle, byref(chan))
        return chan.value
    
    @channel.setter
    def channel (self, channel: int):
        if channel < self.bounds["channel"]["min"] or channel > self.bounds["channel"]["max"]:
            raise Exception("Channel out of bounds.")

        self.device.call(dwf.FDwfAnalogInTriggerChannelSet, self.device.handle, c_int(channel))

    @property
    def condition(self) -> TriggerCondition:
        cond = c_int()
        self.device.call(dwf.FDwfAnalogInTriggerConditionGet, self.device.handle, byref(cond))
        return cond.value

    ###
    # Set the trigger source. 
    # @param src A class property of TriggerCondition.
    ###
    @condition.setter
    def condition(self, condition: TriggerCondition):
        conds = c_int()
        # retrieve possible conditions bit mask
        self.device.call(dwf.FDwfAnalogInTriggerConditionInfo, self.device.handle, byref(conds))
        if (1 << condition) & conds.value:
            self.device.call(dwf.FDwfAnalogInTriggerConditionSet, self.device.handle, c_int(condition))
        else:
            raise Exception("Condition not supported")

    def force(self):
        self.device.call(dwf.FDwfAnalogInTriggerForce, self.device.handle)


class DigilentScopeChannel:
    def __init__(self, device, scope, idx, buffer_size):
        self.device = device
        self.scope = scope
        self.idx = c_int(idx)
        self.__buffer_size = buffer_size
        self.__vertical_division = 1.

    @property
    def enabled(self):
        enabled = c_bool()
        self.device.call(dwf.FDwfAnalogInChannelEnableGet, self.device.handle, self.idx, byref(enabled))
        return enabled.value

    @enabled.setter
    def enabled(self, enabled):
        # already in given mode, nop
        if self.enabled == enabled:
            return

        self.device.call(dwf.FDwfAnalogInChannelEnableSet, self.device.handle, c_bool(enabled))

    @property
    def range(self):
        r = c_double()
        self.device.call(dwf.FDwfAnalogInChannelRangeGet, self.device.handle, self.idx, byref(r))
        return r.value
    def range(self, range):
        self.device.call(dwf.FDwfAnalogInChannelRangeSet,
            self.device.handle, self.idx, c_double(range * _V_DIVS))

    @property
    def vertical_division(self):
        return self.__vertical_division

    @vertical_division.setter
    def vertical_division(self, vdiv):
        self.__vertical_division = vdiv
        self.range = vdiv * _V_DIVS

    @property
    def offset(self):
        offset = c_double()
        self.device.call(dwf.FDwfAnalogInChannelOffsetGet,
            self.device.handle, self.idx, byref(offset))
        return offset.value
    
    @offset.setter
    def offset(self, offset):
        if offset < self.scope.bounds["offset"]["min"] or offset > self.scope.bounds["offset"]["max"]:
            raise Exception("Selected offset is out of bounds")
        self.device.call(dwf.FDwfAnalogInChannelOffsetSet,
            self.device.handle, self.idx, c_double(offset))
        time.sleep(2.0)

    @property
    def data(self):
        self.enabled = True
        buffer = None
        state = c_int()
        self.device.call(dwf.FDwfAnalogInStatus, self.device.handle, c_int(1), byref(state))
        
        u16Read = False
        if u16Read:
            buffer = (c_ushort * self.__buffer_size)()
            self.device.call(dwf.FDwfAnalogInStatusData16,
                self.device.handle, self.idx, buffer, c_int(0), c_int(self.__buffer_size))
            r = self.range
            off = self.offset
            buffer = [float(b * r / 65536. + off)  for b in buffer]
        else:
            buffer = (c_double * self.__buffer_size)()
            self.device.call(dwf.FDwfAnalogInStatusData,
                self.device.handle, self.idx, buffer, c_int(self.__buffer_size))

        u = self.__determine_units()
        freq_step = 1.*u["multiplier"]/self.scope.frequency
        cols = []
        offset = False

        if offset == True:
            # TODO: use when trigger active
            pos = np.arange(0, freq_step * self.__buffer_size//2, freq_step)
            neg = [-i for i in pos]
            cols = np.concatenate((neg,pos))
        else:
            cols = np.arange(0, freq_step * self.__buffer_size, freq_step)

        d = np.fromiter(buffer, dtype = np.float)
        unitstr = "Time (%s)" % u["unit"]
        offr = round(self.offset, 2)
        return {
            "y": d,
            "x": cols,
            "xlabel": unitstr,
            "ylabel": "Voltage (V)",
            "yaxis": np.arange(offr, offr + (self.vertical_division * _V_DIVS), self.vertical_division),
            "yticks": np.arange(offr, offr + (self.vertical_division * _V_DIVS), self.vertical_division),
            "xticks": np.arange(0, self.scope.horizontal_division * _H_DIVS * u["multiplier"], self.scope.horizontal_division * u["multiplier"])
        }

    def __determine_units(self):
        freq = self.scope.frequency
        sample_freq = 1. / freq # frequency to seconds
        total_samples = sample_freq * self.__buffer_size
        units = [{"unit": "ns", "multiplier":1000000000., "tick_step":500}, {"unit": "us", "multiplier":1000000., "tick_step":50}, {"unit": "ms", "multiplier":1000., "tick_step":.5}, {"unit": "s", "multiplier":1., "tick_step":.05}]
        ns_thresh = 0.000000001 
        sf = 1000.
        unit = None

        for u in units:
            unit = u
            if total_samples < ns_thresh * sf:
                break
            sf *= 1000
        
        return unit

    @property
    def voltage(self):
        state = c_int()
        self.device.call(dwf.FDwfAnalogInStatus, self.device.handle, c_int(1), byref(state))
        sample = c_double()
        self.device.call(dwf.FDwfAnalogInStatusSample, self.device.handle, self.idx, byref(sample))
        return sample.value

class DigilentScope:
    channels = []

    class AcquireState:
        Ready = DwfStateReady.value
        Configure = DwfStateConfig.value
        Prefill = DwfStatePrefill.value
        Armed = DwfStateArmed.value
        Wait = DwfStateWait.value
        Triggered = DwfStateTriggered.value
        Running = DwfStateRunning.value
        Done = DwfStateDone.value

    class AcquireMode:
        Single = acqmodeSingle.value
        ScanShift = acqmodeScanShift.value
        ScanScreen = acqmodeScanScreen.value
        Record = acqmodeRecord.value
        SingleShot = acqmodeSingle1.value
        modes = [Single, ScanShift, ScanScreen, Record, SingleShot]

    def __init__(self, device):
        self.device = device
        # ensure hardware is in a default state
        self.device.call(dwf.FDwfAnalogInReset,self.device.handle)
        self.configure()
        self.device.call(dwf.FDwfAnalogInChannelAttenuationSet, self.device.handle, c_int(-1), c_double(1.0))

        # configure hardware buffer size to maximum
        bs = c_int()
        self.device.call(dwf.FDwfAnalogInBufferSizeInfo, self.device.handle, 0, byref(bs))
        self.device.call(dwf.FDwfAnalogInBufferSizeSet, self.device.handle, bs)
        self.__buffer_size = bs.value

        self.bounds = {
            "frequency": {
                "min": -1,
                "max": -1
            },
            "range": {
                "min": -1,
                "max": -1
            },
            "offset": {
                "min": -1,
                "max": -1
            }
        }

        # determine bounds for set calls
        self.__frequency_bounds()
        self.__range_bounds()
        self.__offset_bounds()

        # create channels according to hardware
        for i in range(0, self.channel_count):
            c = DigilentScopeChannel(self.device, self, i, self.__buffer_size)
            self.channels += [c]
            self.__setattr__("channel%d" % (i + 1), c)

        self.trigger = DigilentScopeTrigger(self.device, self)
        self.device.call(dwf.FDwfAnalogInChannelFilterSet, self.device.handle, c_int(-1), filterDecimate)

        self.horizontal_division = .001 # 1ms a div

    @property
    def channel_count(self):
        count = c_int()
        self.device.call(dwf.FDwfAnalogInChannelCount, self.device.handle, byref(count))
        return count.value

    def configure(self):
        self.device.call(dwf.FDwfAnalogInConfigure,self.device.handle, c_int(1), c_int(0))

    def __frequency_bounds(self):
        min = c_double()
        max = c_double()
        self.device.call(dwf.FDwfAnalogInFrequencyInfo,
            self.device.handle, byref(min), byref(max))

        self.bounds["frequency"]["min"] = min.value
        self.bounds["frequency"]["max"] = max.value

    def __offset_bounds(self):
        num_steps = c_int() # ignore?
        offmin = c_double()
        offmax = c_double()

        self.device.call(dwf.FDwfAnalogInChannelOffsetInfo,
            self.device.handle, byref(offmin), byref(offmax), byref(num_steps))

        self.bounds["offset"]["min"] = offmin.value
        self.bounds["offset"]["max"] = offmax.value

    def __range_bounds(self):
        # returns up to possibly 32 steps
        steps = (c_double * 32)()
        num_steps = c_int()
        self.device.call(dwf.FDwfAnalogInChannelRangeSteps,
            self.device.handle, byref(steps), byref(num_steps))

        vmin = c_double()
        vmax = c_double()
        vsteps = c_int()

        self.device.call(dwf.FDwfAnalogInChannelRangeInfo,
            self.device.handle, byref(vmin), byref(vmax), byref(vsteps))

        self.bounds["range"]["min"] = vmin.value
        self.bounds["range"]["max"] = vmax.value

    # set the range for all channels
    # @param range vertical div in volts
    # note, possible range values depend on hardware
    # query self.bounds["range"] for min/max values
    def set_vertical_division(self, vdiv):
         # if a list is given, each channel has its own range
        if isinstance(range, list):
            iter_val = min(len(self.channels), len(range)) # use the smaller length of the two lists
            for i in range(0, iter_val):
                self.channels[i].vertical_division = vdiv[i]
        else:
            for c in self.channels:
                c.vertical_division = vdiv

    def __set_offset(self, idx, offset):
        if offset < self.bounds["offset"]["min"] or offset > self.bounds["offset"]["max"]:
                raise Exception("Selected offset is out of bounds {%f, %f}" % (self.bounds["offset"]["min"], self.bounds["offset"]["max"]))
        self.device.call(dwf.FDwfAnalogInChannelOffsetSet,
                    self.device.handle, c_int(idx), c_double(offset))

    # set the offset for all channels
    # @param offset offset volts - can be a list of offsets
    # note, possible range values depend on hardware
    # query self.bounds["offset"] for min/max values
    def set_offset(self, offset, settle_time=2.0):
        # if a list is given, each channel has its own offset
        if isinstance(offset, list):
            iter_val = min(len(self.channels), len(offset)) # use the smaller length of the two lists
            for i in range(0, iter_val):
                self.__set_offset(i, offset[i])
        else:
            # if only a single float is passed, set the same offset for all channels
            self.__set_offset(-1, offset)

        # settle time is required after the offset
        time.sleep(settle_time)

    @property
    def horizontal_division(self):
        return self.__horizontal_division

    @horizontal_division.setter
    def horizontal_division(self, hdiv):
        self.__horizontal_division = hdiv
        self.frequency = (1./hdiv) * (self.__buffer_size / _H_DIVS)

    @property
    def acquire_mode(self):
        mode = c_int()
        self.device.call(dwf.FDwfAnalogInAcquisitionModeGet, self.device.handle, byref(mode))
        return list(filter(lambda x: x == mode.value, DigilentScope.AcquireMode.modes))[0]

    @acquire_mode.setter
    def acquire_mode(self, mode):
        modes = c_int()
        self.device.call(dwf.FDwfAnalogInAcquisitionModeInfo, self.device.handle, byref(modes))
        # the above call returns a bit mask, AND with the given mode to determine
        # if the provided mode is valid
        if (1 << mode) & modes.value:
            new_mode = c_int(mode)
            dwf.FDwfAnalogInAcquisitionModeSet(
                self.device.handle, new_mode)
        else:
            raise Exception("Acquisition mode not supported")

    @property
    def state(self):
        state = c_int()
        self.device.call(dwf.FDwfAnalogInStatus, self.device.handle, c_int(0), byref(state))
        return state.value

    @property
    def frequency(self):
        freq = c_double()
        self.device.call(dwf.FDwfAnalogInFrequencyGet, self.device.handle, byref(freq))
        return freq.value

    @frequency.setter
    def frequency(self, frequency_hertz):
        if frequency_hertz < self.bounds["frequency"]["min"] or frequency_hertz > self.bounds["frequency"]["max"]:
            raise Exception("Selected frequency is out of bounds")
        self.device.call(dwf.FDwfAnalogInFrequencySet,
            self.device.handle, c_double(frequency_hertz))

    @property
    def data(self):
        if not self.acquire_complete():
            return ([],) * self.channel_count
        tup = ()
        for c in self.channels:
            tup += (c.data,)

        return tup

    def acquire(self, wait_for=None):
        self.device.call(dwf.FDwfAnalogInConfigure, self.device.handle, c_bool(False), c_int(1))

        if wait_for is None:
            return

        if not isinstance(wait_for, DigilentScope.AcquireState):
            raise Exception("Invalid value for wait_for")

        while self.state != wait_for:
            time.sleep(.1)

    def acquire_complete(self):
        return self.state == DigilentScope.AcquireState.Triggered or self.state == DigilentScope.AcquireState.Done

class DigilentDigitalIO:
    class Mode:
        Input = 0
        Output = 1

    def __init__(self, bus, pin_number, name):
        self.bus = bus
        self.name = name
        self.pin_number = pin_number
        self.__mode = DigilentDigitalIO.Mode.Input

    @property
    def mode(self):
        return self.__mode
    
    @mode.setter
    def mode(self, mode):
        if mode != self.__mode:
            n = 1 << self.pin_number
            oe = self.bus.output_enabled
            if mode == DigilentDigitalIO.Mode.Output:
                if not (oe & n):
                    self.bus.output_enabled = oe | n
            else:
                if self.bus.output_enabled & n:
                    self.bus.output_enabled = oe & ~n

            self.__mode == mode

    @property
    def value(self):
        n = 1 << self.pin_number
        if self.bus.value & n:
            return True
        return False

    @value.setter
    def value(self, value):
        # automatically swap to output if user sets value
        if self.mode == DigilentDigitalIO.Mode.Input:
            self.mode = DigilentDigitalIO.Mode.Output

        n = 1 << self.pin_number
        if value:
            self.bus.value |= n
        else:
            self.bus.value &= ~n

class DigilentDigitalIOBus:
    # changes depending on hardware
    # these are provided for typing information
    D0: Optional[DigilentDigitalIO] = None
    D1: Optional[DigilentDigitalIO] = None
    D2: Optional[DigilentDigitalIO] = None
    D3: Optional[DigilentDigitalIO] = None
    D4: Optional[DigilentDigitalIO] = None
    D5: Optional[DigilentDigitalIO] = None
    D6: Optional[DigilentDigitalIO] = None

    def __init__(self, device):
        self.device = device
        dwf.FDwfDigitalIOReset(self.device.handle)
        self.__create_io_props()
    
    def __create_io_props(self):
        # what are the pins and their capabilities?
        pins = c_ulonglong()
        self.device.call(dwf.FDwfDigitalIOOutputEnableInfo64, self.device.handle, byref(pins))
        output_pins = c_ulonglong()
        self.device.call(dwf.FDwfDigitalIOOutputInfo64, self.device.handle, byref(output_pins))
        input_pins = c_ulonglong()
        self.device.call(dwf.FDwfDigitalIOInputInfo64, self.device.handle, byref(input_pins))

        self.pins = pins.value
        self.output_pins = output_pins.value
        self.input_pins = input_pins.value

        # create IO objects
        # i.e. D0-D15 for AD2
        for i in range(0, 64):
            if (1 << i) & pins.value:
                name = "D%d" % i
                self.__setattr__(name, DigilentDigitalIO(self, i, name))

    # what pins are enabled?
    @property
    def output_enabled(self):
        en_state = c_ulonglong()
        self.device.call(dwf.FDwfDigitalIOOutputEnableGet64, self.device.handle, byref(en_state))
        return en_state.value

    @output_enabled.setter
    def output_enabled(self, bitmask):
        bitmask &= self.pins # mask off oob pins
        self.device.call(dwf.FDwfDigitalIOOutputEnableSet64, self.device.handle, c_ulonglong(bitmask))
    
    @property
    def value(self):
        dwf.FDwfDigitalIOStatus(self.device.handle) #must be called before
        in_state = c_ulonglong()
        self.device.call(dwf.FDwfDigitalIOInputStatus64, self.device.handle, byref(in_state))
        return in_state.value

    @value.setter
    def value(self, bitmask):
        bitmask &= self.output_pins # mask off oob pins
        self.device.call(dwf.FDwfDigitalIOOutputSet64, self.device.handle, c_ulonglong(bitmask))

class DigilentPowerNode:
    def __init__(self, device, idx, name, unit, channel):
        self.device = device
        self.channel = channel
        self.idx = c_int(idx)
        self.name = name
        self.unit = unit

        self.__discover_mode()
        self.__get_set_info()

    def __discover_mode(self):
        mode = c_int()
        self.device.call(dwf.FDwfAnalogIOChannelNodeInfo, self.device.handle, self.channel.idx, self.idx, byref(mode))
        self.__mode = mode.value
        #  if self.__mode == analogioEnable.value:
        #     print(" enable", end="")

        # if self.__mode == analogioVoltage.value:
        #     print(" voltage", end="")

        # if self.__mode == analogioCurrent.value:
        #     print(" current", end="")

        # if self.__mode == analogioTemperature.value:
        #     print(" temp", end="")
        
    def __get_set_info(self):
        pmin = c_double()
        pmax = c_double()
        pnum = c_int()
        self.device.call(dwf.FDwfAnalogIOChannelNodeSetInfo, self.device.handle, self.channel.idx, self.idx, byref(pmin), byref(pmax), byref(pnum))

        self.min = pmin.value
        self.max = pmax.value
        self.num_steps = pnum.value

        self.writable = not (self.num_steps == 0 or self.num_steps == 1)

        # print("%s min: %f max: %d steps: %d writable: %d" % (self.name, self.min, self.max, self.num_steps, self.writable))
    
    @property
    def value(self):
        val = c_double()
        self.device.call(dwf.FDwfAnalogIOChannelNodeGet, self.device.handle, self.channel.idx, self.idx, byref(val))
        return val.value

    @value.setter
    def value(self, value):
        if not self.writable:
            raise Exception("Trying to write a read only property")

        self.device.call(dwf.FDwfAnalogIOChannelNodeSet, self.device.handle, self.channel.idx, self.idx, c_double(value))

class DigilentPowerChannel:
    class ChannelType:
        V_PLUS = 0
        V_MINUS = 1
        USB = 2
        AUX = 3
        V_PLUS_MINUS = 4
        types = [V_PLUS, V_MINUS, USB, AUX, V_PLUS_MINUS] 
        types_string = ["V+", "V-", "USB", "Aux", "V+-"]
        var_names = ["vplus", "vminus", "usb", "aux", "vplusminus"]

        def __translate(from_list, value, to_list):
            if value in from_list:
                return to_list[from_list.index(value)]
            return None
        
        @staticmethod
        def label_to_type(label: str):
            translated = DigilentPowerChannel.ChannelType.__translate(DigilentPowerChannel.ChannelType.types_string, label, DigilentPowerChannel.ChannelType.types)
            if translated is None:
                print("Unknown channel type '%s'. Proceed with caution" % label)
                return -1
            return translated
        @staticmethod
        def label_to_varname(label: str):
            translated = DigilentPowerChannel.ChannelType.__translate(DigilentPowerChannel.ChannelType.types_string, label, DigilentPowerChannel.ChannelType.var_names)
            if translated is None:
                raise Exception("Unknown variable '%s'. Cannot generate variable name." % label)
            return translated

    enable: Optional[DigilentPowerNode] = None
    voltage: Optional[DigilentPowerNode] = None
    current: Optional[DigilentPowerNode] = None
    temperature: Optional[DigilentPowerNode] = None

    def __init__(self, device, name, label, idx, power):
        self.device = device
        self.name = name
        self.label = label
        self.idx = c_int(idx)
        self.power = power 
        self.type = self.ChannelType.label_to_type(label)

        nodes = c_int()
        self.device.call(dwf.FDwfAnalogIOChannelInfo, self.device.handle, self.idx, byref(nodes))
        self.__node_count = nodes.value

        nodes = []

        for i in range(0, self.__node_count):
            nodeName = (c_char * 32)()
            units = (c_char * 16)()
            self.device.call(dwf.FDwfAnalogIOChannelNodeName, self.device.handle, self.idx, c_int(i), nodeName, units)
            nodeName = str(nodeName.value, encoding="utf-8")
            unit = str(units, encoding="utf-8")
            n = DigilentPowerNode(self.device, i, nodeName.lower(), unit, self)
            nodes += [n]
            self.__setattr__(nodeName.lower(), n)

        # used as a filter in __setattr__ to wrap assigns to objects we care about
        self.nodes = nodes

    def __setattr__(self, name, value):
        nodes = []
        if "nodes" in self.__dict__.keys():
            nodes = self.nodes
        if name in [n.name for n in nodes]:
            self.__dict__[name].value = value
            print("CUSTOM SET ATTR %s %s" % (name,value))
            return
        self.__dict__[name] = value

    def __getattr__(self, name: str):
        self.power._fetch_status()
        nodes = []
        if "nodes" in self.__dict__.keys():
            nodes = self.nodes
        if name in [n.name for n in nodes]:
            return self.__dict__[name].value
        return self.__dict__[name]

class DigilentPowerMaster:
    def __init__(self, device: DigilentDevice, power):
        self.device = device
        self.power = power

    @property    
    def enable(self) -> bool:
        self.power._fetch_status()
        en = c_int()
        self.device.call(dwf.FDwfAnalogIOEnableStatus, self.device.handle, byref(en))
        return en.value > 0

    @enable.setter
    def enable(self, value: bool):
        self.device.call(dwf.FDwfAnalogIOEnableSet, self.device.handle, c_bool(value))

class DigilentPower:
    master: Optional[DigilentPowerMaster] = None
    vplus: Optional[DigilentPowerChannel] = None
    vminus: Optional[DigilentPowerChannel] = None
    usb: Optional[DigilentPowerChannel] = None
    aux: Optional[DigilentPowerChannel] = None
    vplusminus: Optional[DigilentPowerChannel] = None
    
    def __init__(self, device):
        self.device = device
        # reset interfaces
        self.device.call(dwf.FDwfAnalogIOReset, self.device.handle)

        self._fetch_status()

        mEnSupp = c_int()
        mReadable = c_int()
        self.device.call(dwf.FDwfAnalogIOEnableInfo, self.device.handle, byref(mEnSupp), byref(mReadable))

        if mEnSupp.value > 0:
            self.master = DigilentPowerMaster(self.device)
        
        self.__create_channels()

    def __create_channels(self):
        chan_count = c_int()
        self.device.call(dwf.FDwfAnalogIOChannelCount, self.device.handle, byref(chan_count))

        for c in range(0, chan_count.value):
            name = (c_char * 32)() # max name is 32 bytes
            label = (c_char * 16)() # max label is 16 bytes
            self.device.call(dwf.FDwfAnalogIOChannelName, self.device.handle, c_int(c), byref(name), byref(label))
            
            name = str(name.value, encoding="utf-8")
            label = str(label.value, encoding="utf-8")

            self.__setattr__(DigilentPowerChannel.ChannelType.label_to_varname(label), DigilentPowerChannel(self.device, name, label, c, self))

    def _fetch_status(self):
        # read status from dev..
        self.device.call(dwf.FDwfAnalogIOStatus, self.device.handle)

class DigilentI2C:
    class I2CDataFrame:
        start_detected: bool
        restart_detected: bool
        stop_detected: bool
        data: list[int]
        error: bool
        def __init__(self, start, stop, data, dmax, nak_status):
            self.start_detected = start.value == 1
            self.restart_detected = start.value == 2
            self.stop_detected = stop.value == 1
            self.data = [d for idx, d in enumerate(data) if idx < dmax.value]
            self.error = nak_status.value < 0 # neg indicates error

    def __init__(self, device):
        self.device = device
        self.device.call(dwf.FDwfDigitalI2cReset, self.device.handle)
        self.baud = 100000
        self.__sda = None
        self.__scl = None

    def fix_lockup(self):
        unlocked = c_int()
        self.device.call(dwf.FDwfDigitalI2cClear, self.device.handle, byref(unlocked))
        if not unlocked:
            raise Exception("Could not resolve I2C lock up.")

    def __check_initd(self):
        if self.__sda is None or self.__scl is None:
            raise Exception("SCL and SDA are not set. Set SCL and SDA pins before use.")

    @property
    def scl_pin(self) -> DigilentDigitalIO:
        return self.__scl
    
    @scl_pin.setter
    def scl_pin(self, pin: DigilentDigitalIO):
        self.device.call(dwf.FDwfDigitalI2cSclSet, self.device.handle, pin.idx)
        self.__scl = pin
        pass

    @property
    def sda_pin(self) -> DigilentDigitalIO:
        return self.__sda
    
    @sda_pin.setter
    def sda_pin(self, pin: DigilentDigitalIO):
        self.device.call(dwf.FDwfDigitalI2cSdaSet, self.device.handle, pin.idx)
        self.__sda = pin

    def set_pins(self, sda: DigilentDigitalIO, scl: DigilentDigitalIO):
        self.sda = sda
        self.scl = scl

    @property
    def baud(self):
        return self.__baud

    @baud.setter
    def baud(self, baudHz):
        self.device.call(dwf.FDwfDigitalI2cRateSet, self.device.handle, c_double(baudHz))
        self.__baud = baudHz

    def write(self, devAddress, registerAddress, data):
        self.__check_initd()

        devAddress <= 1

        txBuffer = (c_ubyte * len(data + 1))()
        txBuffer[0] = c_ubyte(registerAddress)
        for i in range(1, len(data)):
            txBuffer[i] = c_ubyte(data[i])
        
        nak = c_int()

        self.device.call(dwf.FDwfDigitalI2cWrite, self.device.handle, c_ubyte(devAddress), txBuffer, c_int(len(data)), byref(nak))

        if nak != 0:
            raise Exception("One or more bytes were not ACK'd.")

    def read(self, devAddress, registerAddress, rxLength) -> list[int]:
        self.__check_initd()
        devAddress <= 1
        registerAddress = c_ubyte(registerAddress)

        rxBuffer = (c_ubyte * rxLength)()

        nak = c_int()

        self.device.call(dwf.FDwfDigitalI2cWriteRead, self.device.handle, registerAddress, c_int(1), rxBuffer, c_int(rxLength), byref(nak))

        if nak != 0:
            raise Exception("One or more bytes were not ACK'd.")

        return [d.value for d in rxBuffer]

    # places the device into "spy" mode, listening for values sent on the bus.
    def decode(self) -> I2CDataFrame:
        self.device.call(dwf.FDwfDigitalI2cSpyStart, self.device.handle)

        start = c_int()
        stop = c_int()
        dMax = c_int()
        nak = c_int()

        dataBuffer = (c_ubyte * 256)()

        self.device.call(self.device.handle, byref(start), byref(stop), dataBuffer, byref(dMax), byref(nak))

        return DigilentI2C.I2CDataFrame(start, stop, dataBuffer, dMax, nak)