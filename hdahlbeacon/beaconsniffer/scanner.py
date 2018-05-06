from bluepy.btle import Scanner, DefaultDelegate

sensorTags = ['b0:b4:48:be:9a:03', 'othertag']

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.scanner = None
        self.timeout = 0

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)
        # print("Handlediscovery was called with device %s" % dev.addr)    
        if dev.addr in sensorTags:
            # print("Received data from sensortag!")
            # dev.getScanData()
            
            for (adtype, desc, value) in dev.getScanData():
                print("new sensortag values ::  %s = %s || Adtype: %s" % (desc, value, adtype))
            # #dev.scanData = {}
            # self.scanner.timeout = 1
            # if self.scanner != None:
            #     self.scanner.stop()

scanner = Scanner().withDelegate(ScanDelegate())
scanner.delegate.scanner = scanner # I know, this makes me cringe too
scanner.timeout = 10
devices = scanner.scan(scanner.timeout, passive=True)

for dev in devices:
    print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
    for (adtype, desc, value) in dev.getScanData():
        print("  %s = %s || Adtype: %s" % (desc, value, adtype))
