## BLE Scanner

### Gettins started

1. If youre familiar with python at all, getting started here is pretty much the same way as everywhere. $pip install -r requirements.txt
2. Had to run the following shell script to avoid accessing bluetooth device as root:
```
sudo setcap 'cap_net_raw,cap_net_admin+eip' PATH/TO/LIB/python3.5/site-packages/bluepy/bluepy-helper
```
