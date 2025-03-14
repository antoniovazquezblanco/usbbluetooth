#include <usbbluetooth_io.h>

#include <utils.h>
#include <usbbluetooth_log.h>

#define TIMEOUT 1000

typedef enum
{
    HCI_HDR_TYPE_ACK = 0,
    HCI_HDR_TYPE_CMD = 1,
    HCI_HDR_TYPE_ACL = 2,
    HCI_HDR_TYPE_SYN = 3,
    HCI_HDR_TYPE_EVT = 4,
    HCI_HDR_TYPE_VENDOR = 14,
    HCI_HDR_TYPE_LINK_CONTROL = 15,
} hci_hdr_type_t;

usbbluetooth_status_t USBBLUETOOTH_CALL usbbluetooth_open(usbbluetooth_device_t *dev)
{
    // Open the device and get a handle...
    int err = libusb_open(dev->device.usb, &dev->context.usb->handle);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = _dev_find_bluetooth_interface(dev->device.usb, &dev->context.usb->interface_num);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = libusb_set_auto_detach_kernel_driver(dev->context.usb->handle, 1);
    if (err < LIBUSB_SUCCESS && err != LIBUSB_ERROR_NOT_SUPPORTED)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = libusb_claim_interface(dev->context.usb->handle, dev->context.usb->interface_num);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = _dev_find_evt_ep(dev->device.usb, &dev->context.usb->epnum_evt);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = _dev_find_acl_in_ep(dev->device.usb, &dev->context.usb->epnum_acl_in);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    err = _dev_find_acl_out_ep(dev->device.usb, &dev->context.usb->epnum_acl_out);
    if (err < LIBUSB_SUCCESS)
        return USBBLUETOOTH_STATUS_ERR_UNK;

    return USBBLUETOOTH_STATUS_OK;
}

void USBBLUETOOTH_CALL usbbluetooth_close(usbbluetooth_device_t *dev)
{
    if (dev->context.usb->handle == NULL)
        return;
    libusb_release_interface(dev->context.usb->handle, dev->context.usb->interface_num);
    libusb_close(dev->context.usb->handle);
    dev->context.usb->handle = NULL;
}

usbbluetooth_status_t USBBLUETOOTH_CALL usbbluetooth_write(usbbluetooth_device_t *dev, uint8_t *data, uint16_t size)
{
    if (dev->context.usb->handle == NULL)
        return USBBLUETOOTH_STATUS_ERR_DEVICE_CLOSED;

    uint8_t type = data[0];
    size--;
    int err = LIBUSB_SUCCESS;
    switch (type)
    {
    case HCI_HDR_TYPE_CMD:
        err = libusb_control_transfer(dev->context.usb->handle,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                                      0x00, 0x0000, 0x0000, &data[1], size, TIMEOUT);
        return (err < LIBUSB_SUCCESS) ? USBBLUETOOTH_STATUS_ERR_UNK : USBBLUETOOTH_STATUS_OK;
    case HCI_HDR_TYPE_ACL:
        err = libusb_bulk_transfer(dev->context.usb->handle, dev->context.usb->epnum_acl_out, &data[1], size, NULL, TIMEOUT);
        return (err < LIBUSB_SUCCESS) ? USBBLUETOOTH_STATUS_ERR_UNK : USBBLUETOOTH_STATUS_OK;
    default:
        return USBBLUETOOTH_STATUS_ERR_UNK;
    }
}

int _read_data(usbbluetooth_device_t *dev, uint8_t *data, uint16_t *size)
{
    int recevd = 0;
    int err = libusb_bulk_transfer(dev->context.usb->handle, dev->context.usb->epnum_acl_in, &data[1], (*size) - 1, &recevd, TIMEOUT);
    if (err < LIBUSB_SUCCESS)
        return err;
    data[0] = HCI_HDR_TYPE_ACL;
    *size = recevd + 1;
    return LIBUSB_SUCCESS;
}

int _read_evts(usbbluetooth_device_t *dev, uint8_t *data, uint16_t *size)
{
    int recevd = 0;
    int err = libusb_interrupt_transfer(dev->context.usb->handle, dev->context.usb->epnum_evt, &data[1], (*size) - 1, &recevd, TIMEOUT);
    if (err < LIBUSB_SUCCESS)
        return err;
    data[0] = HCI_HDR_TYPE_EVT;
    *size = recevd + 1;
    return LIBUSB_SUCCESS;
}

usbbluetooth_status_t USBBLUETOOTH_CALL usbbluetooth_read(usbbluetooth_device_t *dev, uint8_t *data, uint16_t *size)
{
    usbbluetooth_log_debug("usbbluetooth_read");

    if (dev->context.usb->handle == NULL)
        return USBBLUETOOTH_STATUS_ERR_DEVICE_CLOSED;

    int err = _read_evts(dev, data, size);
    usbbluetooth_log_debug("_read_evts[err=%d, size=%d]", err, *size);
    if (err != LIBUSB_ERROR_TIMEOUT)
        return (err == LIBUSB_SUCCESS) ? USBBLUETOOTH_STATUS_OK : USBBLUETOOTH_STATUS_ERR_UNK;

    err = _read_data(dev, data, size);
    usbbluetooth_log_debug("_read_data[err=%d, size=%d]", err, *size);
    if (err != LIBUSB_ERROR_TIMEOUT)
        return (err == LIBUSB_SUCCESS) ? USBBLUETOOTH_STATUS_OK : USBBLUETOOTH_STATUS_ERR_UNK;

    // No data to retrieve...
    *size = 0;
    return USBBLUETOOTH_STATUS_OK;
}
