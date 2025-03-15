# USB Bluetooth

[![Build](https://github.com/antoniovazquezblanco/usbbluetooth/actions/workflows/build.yml/badge.svg)](https://github.com/antoniovazquezblanco/usbbluetooth/actions/workflows/build.yml)
[![PyPI](https://img.shields.io/pypi/v/usbbluetooth)](https://pypi.org/project/usbbluetooth/)
[![Snyk](https://snyk.io/advisor/python/usbbluetooth/badge.svg)](https://snyk.io/advisor/python/usbbluetooth)

Take full control of your USB Bluetooth controllers! 

If you want to use this library with Scapy, please checkout [scapy-usbbluetooth](https://pypi.org/project/scapy-usbbluetooth/).

In order to use the library in python just install the Python package.
You may install the package from Pypi:
```bash
pip install usbbluetooth
```

Once installed, you may follow the example below as a reference on how to list and interact with devices:

```Python
#!/usr/bin/env python

import usbbluetooth

# Get a list of all the available devices
devices = usbbluetooth.list_devices()
for dev in devices:
    print(dev)

# Open the device
with devices[0] as dev:
    # Send a reset command
    dev.write(b"\x01\x03\x0c\x00")
    # Read the respose
    response = dev.read()
    print(response)
```
