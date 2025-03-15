# API for Python

If you want to use this library with Scapy, please checkout [scapy-usbbluetooth](https://github.com/antoniovazquezblanco/scapy-usbbluetooth), otherwise, the `usbbluetooth` library can be used via a Python API.

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
