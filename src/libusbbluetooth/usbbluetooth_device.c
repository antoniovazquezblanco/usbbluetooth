#include <usbbluetooth_device.h>

#include <usbbluetooth_log.h>
#include <utils.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int _count_bluetooth_devices(libusb_device **list, int *num);
static usbbluetooth_device_t *_dev_from_libusb(libusb_device *dev);

usbbluetooth_status_t USBBLUETOOTH_CALL usbbluetooth_get_device_list(usbbluetooth_device_t ***list_ptr)
{
    usbbluetooth_log_debug("usbbluetooth_get_device_list");

    // Check list parameter
    if (list_ptr == NULL)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    // Get a list of all devices...
    libusb_device **devs_internal;
    int r = libusb_get_device_list(NULL, &devs_internal);
    usbbluetooth_log_debug("libusb_get_device_list[r=%d]", r);
    if (r < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    // Count the number of bluetooth devices...
    int num_devs = 0;
    r = _count_bluetooth_devices(devs_internal, &num_devs);
    usbbluetooth_log_debug("_count_bluetooth_devices[r=%d, n=%d]", r, num_devs);
    if (r < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    // Create a new list!
    *list_ptr = calloc((size_t)num_devs + 1, sizeof(struct usbbluetooth_device_t *));
    if (*list_ptr == NULL)
        return USBBLUETOOTH_STATUS_ERR_NOMEM;

    // Terminate the list...
    (*list_ptr)[num_devs] = NULL;

    // Get the list
    usbbluetooth_device_t **list = *list_ptr;

    // Iterate again...
    libusb_device *dev;
    for (int i = 0, pos = 0; (dev = devs_internal[i]) != NULL; i++)
    {
        bool is_bt = false;
        if (_is_bluetooth_device(dev, &is_bt) == LIBUSB_SUCCESS && is_bt)
        {
            list[pos++] = usbbluetooth_reference_device(_dev_from_libusb(dev));
        }
    }

    // Cleanup
    libusb_free_device_list(devs_internal, 1);

    return USBBLUETOOTH_STATUS_OK;
}

static int _count_bluetooth_devices(libusb_device **list, int *num)
{
    // Reset the counter...
    *num = 0;

    // Iterate all devices...
    libusb_device *dev;
    for (int i = 0; (dev = list[i]) != NULL; i++)
    {
        // Check if device has a Bluetooth interface...
        bool is_bt = false;
        int r = _is_bluetooth_device(dev, &is_bt);
        if (r == LIBUSB_ERROR_NOT_FOUND)
            is_bt = false;
        else if (r < LIBUSB_SUCCESS)
            return r;
        if (is_bt)
            (*num)++;
    }
    return LIBUSB_SUCCESS;
}

static usbbluetooth_device_t *_dev_from_libusb(libusb_device *dev)
{
    usbbluetooth_log_debug("_dev_from_libusb[dev=%p]", dev);
    usbbluetooth_device_t *btdev = calloc(1, sizeof(usbbluetooth_device_t));
    btdev->ref_count = 0;
    btdev->type = USBBLUETOOTH_DEVICE_TYPE_USB;
    btdev->device.usb = libusb_ref_device(dev);
    usbbluetooth_device_ctx_usb_t *ctx = calloc(1, sizeof(usbbluetooth_device_ctx_usb_t));
    memset(ctx, 0, sizeof(usbbluetooth_device_ctx_usb_t));
    btdev->context.usb = ctx;
    return btdev;
}

void USBBLUETOOTH_CALL usbbluetooth_free_device_list(usbbluetooth_device_t ***list)
{
    // Cannot clean empty list...
    if (list == NULL || *list == NULL)
        return;

    // Iterate each device...
    usbbluetooth_device_t *dev;
    for (int i = 0; (dev = (*list)[i]) != NULL; i++)
        usbbluetooth_unreference_device(&dev);

    // Free the list
    free(*list);

    // Set the pointer to null...
    *list = NULL;
}

usbbluetooth_device_t *USBBLUETOOTH_CALL usbbluetooth_reference_device(usbbluetooth_device_t *dev)
{
    usbbluetooth_log_debug("usbbluetooth_reference_device[dev=%p]\n", dev);
    dev->ref_count++;
    return dev;
}

void USBBLUETOOTH_CALL usbbluetooth_unreference_device(usbbluetooth_device_t **dev_ptr)
{
    if (dev_ptr == NULL || *dev_ptr == NULL)
        return;

    usbbluetooth_device_t *dev = *dev_ptr;
    dev->ref_count--;

    if (dev->ref_count == 0)
    {
        libusb_unref_device(dev->device.usb);
        free(dev->context.usb);
        free(dev);
        *dev_ptr = NULL;
    }
}

void USBBLUETOOTH_CALL usbbluetooth_device_vid_pid(usbbluetooth_device_t *dev, uint16_t *vid, uint16_t *pid)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev->device.usb, &desc);
    *vid = desc.idVendor;
    *pid = desc.idProduct;
}

char *_usb_get_descriptor_ascii(libusb_device_handle *dev_handle, uint8_t desc_num)
{
    if (dev_handle == NULL)
        return NULL;
    char tmp[256];
    int r = libusb_get_string_descriptor_ascii(dev_handle, desc_num, (uint8_t *)tmp, sizeof(tmp));
    if (r < LIBUSB_SUCCESS)
        return NULL;
    tmp[r] = 0;
    return strdup(tmp);
}

char *USBBLUETOOTH_CALL usbbluetooth_device_manufacturer(usbbluetooth_device_t *dev)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev->device.usb, &desc);
    if (desc.iManufacturer == 0)
        return NULL;
    return _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iManufacturer);
}

char *USBBLUETOOTH_CALL usbbluetooth_device_product(usbbluetooth_device_t *dev)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev->device.usb, &desc);
    if (desc.iProduct == 0)
        return NULL;
    return _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iProduct);
}

char *USBBLUETOOTH_CALL usbbluetooth_device_serial_num(usbbluetooth_device_t *dev)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev->device.usb, &desc);
    if (desc.iSerialNumber == 0)
        return NULL;
    return _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iSerialNumber);
}

char *USBBLUETOOTH_CALL usbbluetooth_device_description(usbbluetooth_device_t *dev)
{
    // Get USB descriptor
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev->device.usb, &desc);
// Create a tmp string with VID and PID
#define MAX_LEN 256
    uint16_t len_rem = MAX_LEN;
    char tmp[MAX_LEN];
    len_rem -= snprintf(tmp, len_rem, "VID=0x%04x PID=0x%04x", desc.idVendor, desc.idProduct);
    // Add accesory info if available
    char *manuf = _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iManufacturer);
    if (manuf)
    {
        len_rem -= snprintf(tmp + strlen(tmp), len_rem, " %s", manuf);
        free(manuf);
    }
    char *prod = _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iProduct);
    if (prod)
    {
        len_rem -= snprintf(tmp + strlen(tmp), len_rem, " %s", prod);
        free(prod);
    }
    char *sernum = _usb_get_descriptor_ascii(dev->context.usb->handle, desc.iSerialNumber);
    if (sernum)
    {
        len_rem -= snprintf(tmp + strlen(tmp), len_rem, " %s", sernum);
        free(sernum);
    }
    return strdup(tmp);
}
