## Personal work on Contiki-ng

This repo is part of py process of understanding Contiki OS and the new fork of contiki - contiki-ng. I have taken and altered a couple of examples from the examples in contiki-ng and altered them to test various things.

### Getting started
1. This repo uses submodules. Clone this repo recursively or remember to initialize the submodules
2. You need GCC for ARM toolchain
3. Also some tools:
```
$ sudo apt install build-essential git ntp srecord
```
4. See the subfolders for more specific instructions per sub-project
5. To flash, TI Uniflash must be installed:
http://processors.wiki.ti.com/index.php/Category:CCS_UniFlash
Deps : http://processors.wiki.ti.com/index.php/Linux_Host_Support_CCSv6#Ubuntu_16.04_64bit
6. Note that there are board configuration files in the repo (*.ccxml) - these are used for connecting to devices before flashing - the Debugger one is meant for the sensortag debugger shield. There may be an issue here based on the platform, and they can be regenerated using Uniflash.
7. See the .bashrcdahl shell script and alter the variable at the top of the document for wherever the path to the repo is. Source the script and you should be able to run the different utility scripts

### Scripts
See the .bashrcdahl shell script. Alter the variable for where in the fs this repo is located and source the scipt. This will provide some different utilities for compilation and flashing of sensortags:

ngmake : Builds a subproject - Call with argument "subfolder"

nglogin : logs into the connected node to see uart output but may also give shell prompt if that is enabled

ngmakerouter : builds the hdahlrouter subfolder for getting an RPL router node

ngconnectrouter : connects to router node using tunslip6 to bridge it to the internet 

ngmakeclean : Cleans a subfolder

### Sub-projects

**hdahlapp** : Very simple application based on hello world, but it is silent and does nothing. It includes the shell module so one can log connect to the tags and inspect them.

**hdahlrouter** : Based on the RPL router. Flash this program to the node and connect to it using the tun6 tool. This will let the node work as an RPL router and bridge it to the internet, such that we have a compatible root IPv6 router on the RPL DAG.

**hdahlbeacon** : Subproject for running sensortags as beacons. Also has some sniffer programs which may be run on a gateway
