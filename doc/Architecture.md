# Architecture

Here you may find how the Bluetooth architecture is defined and how this software fits on it.

## Physical architecture

Bluetooth physicall architecture is separated into two main components.

- Controller: Is a hardware specialized device that handles radiowave operations and time critical stuff. You would normally find a controller chip embeded in your PC or you may have a dongle with a Bluetooth controller chip.
- Host: The machine that uses a controller to communicate to other Bluetooth devices. This would normally be a piece of software that runs in your PC.

The physicall connection between a Host and a Controller can be made via various transport layers:

- USB
- Serial/UART

For those two components to communicate, the Bluetooth specification defines a _Host Controller Interface_ (HCI). HCI is a protocol that standardizes the usage of all Bluetooth controllers.

## USB Bluetooth software architecture

USB Bluetooth is a small part of the Bluetooth Host definition.

This small piece of software allows an user to list the Bluetooth controllers connected to a host and to communicate with those controllers via a common interface using the HCI protocol.

This common interface is an implementation of the USB transport protocol as defined by the Bluetooth standard.

In order to list and communicate with USB devices, USB Bluetooth uses LibUSB as USB driver.

![Architecture](Architecture.svg)
