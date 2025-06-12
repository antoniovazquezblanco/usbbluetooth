# Documentation

This project consist of a library that allows you to enumerate and control USB Bluetooth devices.

There is also an example binary compiled to test the library in your platform, it is called `usbbt`.
The `usbbt` executable allows you to list available devices and to send them a reset command.


## Plaform quirks

In Windows you may have to install WinUSB driver in your device using Zadig. Otherwise, UsbBluetooth may not be able to take control of your device.


## Usage

This library has been programmed in C. You can find information on how to compile, install and use it in the [C API document](API_C.md).

You can also find documentation for the [Python API](API_Python.md) or the [C# API](API_CSharp.md).


## Architecture

You may find more information about the design and architecture of this software in the [Architecture](Architecture.md) document.
