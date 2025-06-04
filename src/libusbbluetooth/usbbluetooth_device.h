#ifndef _H_USBBLUETOOTH_DEVICE_
#define _H_USBBLUETOOTH_DEVICE_

#include <usbbluetooth_api.h>
#include <usbbluetooth_err.h>
#include <libusb.h>
#include <libserialport.h>
#include <stdint.h>

/**
 * Device type enumeration.
 */
typedef enum
{
    USBBLUETOOTH_DEVICE_TYPE_USB = 0,
    USBBLUETOOTH_DEVICE_TYPE_SERIAL = 1
} usbbluetooth_device_type_t;

/**
 * USB device context structure.
 */
typedef struct
{
    libusb_device_handle *handle;
    uint8_t interface_num;
    uint8_t epnum_evt;
    uint8_t epnum_acl_in;
    uint8_t epnum_acl_out;
} usbbluetooth_device_ctx_usb_t;

/**
 * USB Bluetooth device struct.
 */
typedef struct
{
    uint8_t ref_count;
    usbbluetooth_device_type_t type;
    union
    {
        libusb_device *usb;
        struct sp_port *ser;
    } device;
    union
    {
        usbbluetooth_device_ctx_usb_t *usb;
    } context;
} usbbluetooth_device_t;

/**
 * List USB Bluetooth devices found in the machine.
 */
usbbluetooth_status_t USBBLUETOOTH_CALL usbbluetooth_get_device_list(usbbluetooth_device_t ***list);

/**
 * Free a list of USB Bluetooth devices.
 */
void USBBLUETOOTH_CALL usbbluetooth_free_device_list(usbbluetooth_device_t ***list);

/**
 * Get a reference to a Bluetooth device.
 */
usbbluetooth_device_t *USBBLUETOOTH_CALL usbbluetooth_reference_device(usbbluetooth_device_t *dev);

/**
 * Unreference the device to free memory.
 */
void USBBLUETOOTH_CALL usbbluetooth_unreference_device(usbbluetooth_device_t **dev);

/**
 * Get device VID and PID.
 */
void USBBLUETOOTH_CALL usbbluetooth_device_vid_pid(usbbluetooth_device_t *dev, uint16_t *vid, uint16_t *pid);

/**
 * Get device manufacturer description.
 * Due to current libusb implementations, it may be required for the device to be open to be able to retrieve this information.
 * The user is reponsible for freeing the string after its use.
 */
char *USBBLUETOOTH_CALL usbbluetooth_device_manufacturer(usbbluetooth_device_t *dev);

/**
 * Get device product description.
 * Due to current libusb implementations, it may be required for the device to be open to be able to retrieve this information.
 * The user is reponsible for freeing the string after its use.
 */
char *USBBLUETOOTH_CALL usbbluetooth_device_product(usbbluetooth_device_t *dev);

/**
 * Get device serial number.
 * Due to current libusb implementations, it may be required for the device to be open to be able to retrieve this information.
 * The user is reponsible for freeing the string after its use.
 */
char *USBBLUETOOTH_CALL usbbluetooth_device_serial_num(usbbluetooth_device_t *dev);

/**
 * Get an user friendly representation of a device in text form.
 * The user is reponsible for freeing the string after its use.
 */
char *USBBLUETOOTH_CALL usbbluetooth_device_description(usbbluetooth_device_t *dev);

#endif
