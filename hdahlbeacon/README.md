## cc26xx beacon
===========

This folder contains the necessities of getting a sensortag beacon project up and running.

**The project contains three parts**
- Python BLE sniffer
- React Native BLE sniffer
- Sensortag beacon


### Getting started with Embedded beacons


### Getting started with Python BLE sniffer

1. If youre familiar with python at all, getting started here is pretty much the same way as everywhere. $pip install -r requirements.txt
2. Had to run the following shell script to avoid accessing bluetooth device as root:
```
sudo setcap 'cap_net_raw,cap_net_admin+eip' PATH/TO/LIB/python3.5/site-packages/bluepy/bluepy-helper
```
3. The sniffer is based on bluepy. See the scanner.py example to get started and the [bluepy documentation](http://ianharvey.github.io/bluepy-doc/index.html)

### Getting started with React Native sniffer
todo
