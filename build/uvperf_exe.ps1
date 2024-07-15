#!/usr/bin/env pwsh

./uvperf
./check_usb.ps1
python3 "../find_usb.py"

./uvperf -v 0x1004 -p 0x61A1 -i 0 -a 0 -e 0x81 -t 1000 -l 30000 -S
