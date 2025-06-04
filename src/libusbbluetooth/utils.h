#ifndef _H_UTILS_
#define _H_UTILS_

#include <libusb.h>
#include <stdbool.h>

/**
 * Check if endpoint has the specifified direction.
 */
bool _libusb_is_ep_dir(const struct libusb_endpoint_descriptor* ep, enum libusb_endpoint_direction dir);

bool _libusb_device_descriptor_is_bluetooth(struct libusb_device_descriptor *desc);
bool _libusb_interface_is_bluetooth(const struct libusb_interface_descriptor *interface);
bool _libusb_interface_has_bluetooth_altsetting(const struct libusb_interface *iface);
bool _libusb_configuration_has_bluetooth_interface(struct libusb_config_descriptor *config);
int _libusb_is_bluetooth_device(libusb_device *dev, bool *is_bt);

int _libusb_dev_find_bluetooth_interface(libusb_device *dev, uint8_t *interface_number);
int _libusb_dev_find_evt_ep(libusb_device *dev, uint8_t *epnum);
int _libusb_dev_find_acl_in_ep(libusb_device *dev, uint8_t *epnum);
int _libusb_dev_find_acl_out_ep(libusb_device *dev, uint8_t *epnum);

#endif
