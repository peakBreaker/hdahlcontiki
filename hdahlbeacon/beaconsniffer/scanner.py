from bluepy.btle import Scanner, DefaultDelegate
import requests

sensorTags = ['b0:b4:48:be:9a:03', 'othertag']

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.scanner = None
        self.timeout = 0

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr in sensorTags:
            for (adtype, desc, value) in dev.getScanData():
                print("new sensortag values ::  %s = %s || Adtype: %s" % (desc, value, adtype))
                if adtype == 255:
                    raw_values = bytearray.fromhex(value).decode().split(',')
                    sensor_values = {'x': raw_values[0],
                                     'y': raw_values[1],
                                     'z': raw_values[2],
                                     'id': dev.addr}
                    print("sending to server")
                    print(sensor_values)
                    

scanner = Scanner().withDelegate(ScanDelegate())
scanner.delegate.scanner = scanner # I know, this makes me cringe too
scanner.timeout = 10
devices = scanner.scan(scanner.timeout, passive=True)

# for dev in devices:
#     print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
#     for (adtype, desc, value) in dev.getScanData():
#         print("  %s = %s || Adtype: %s" % (desc, value, adtype))
