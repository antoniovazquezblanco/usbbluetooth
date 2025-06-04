#include <utils.h>

bool _libusb_is_ep_dir(const struct libusb_endpoint_descriptor *ep, enum libusb_endpoint_direction dir)
{
    return (ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == dir;
}

bool _libusb_device_descriptor_is_bluetooth(struct libusb_device_descriptor *desc)
{
    return (desc->bDeviceClass == LIBUSB_CLASS_WIRELESS) &&
           (desc->bDeviceSubClass == 0x01 /* SUBCLASS_RF_CONTROLLER */) &&
           (desc->bDeviceProtocol == 0x01 /* PROTOCOL_BLUETOOTH_PRIMARY_CONTROLLER */);
}

bool _libusb_interface_is_bluetooth(const struct libusb_interface_descriptor *interface)
{
    return (interface->bInterfaceClass == LIBUSB_CLASS_WIRELESS) &&
           (interface->bInterfaceSubClass == 0x01 /* SUBCLASS_RF_CONTROLLER */) &&
           (interface->bInterfaceProtocol == 0x01 /* PROTOCOL_BLUETOOTH_PRIMARY_CONTROLLER */);
}

bool _libusb_interface_has_bluetooth_altsetting(const struct libusb_interface *iface)
{
    // Check all altsettings...
    for (int altsetting_idx = 0; altsetting_idx < iface->num_altsetting; altsetting_idx++)
    {
        const struct libusb_interface_descriptor *altsetting = &iface->altsetting[altsetting_idx];
        if (_libusb_interface_is_bluetooth(altsetting))
            return true;
    }
    return false;
}

bool _libusb_configuration_has_bluetooth_interface(struct libusb_config_descriptor *config)
{
    for (int iface_idx = 0; iface_idx < config->bNumInterfaces; iface_idx++)
    {
        const struct libusb_interface *iface = &config->interface[iface_idx];
        if (_libusb_interface_has_bluetooth_altsetting(iface))
            return true;
    }
    return false;
}

int _libusb_is_bluetooth_device(libusb_device *dev, bool *is_bt)
{
    *is_bt = false;

    // Check device descriptor...
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < LIBUSB_SUCCESS)
        return r;
    if (_libusb_device_descriptor_is_bluetooth(&desc))
    {
        *is_bt = true;
        return LIBUSB_SUCCESS;
    }

    // For each configuration, check all interfaces...
    for (int config_idx = 0; config_idx < desc.bNumConfigurations; config_idx++)
    {
        struct libusb_config_descriptor *config;
        r = libusb_get_config_descriptor(dev, config_idx, &config);
        if (r < LIBUSB_SUCCESS)
            return r;
        if (_libusb_configuration_has_bluetooth_interface(config))
        {
            *is_bt = true;
            return LIBUSB_SUCCESS;
        }
    }
    // Not a bluetooth device...
    return LIBUSB_SUCCESS;
}

int _libusb_dev_find_bluetooth_interface(libusb_device *dev, uint8_t *interface_number)
{
    // Check device descriptor...
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < LIBUSB_SUCCESS)
        return r;

    // For each configuration, check all interfaces...
    for (int config_idx = 0; config_idx < desc.bNumConfigurations; config_idx++)
    {
        struct libusb_config_descriptor *config;
        r = libusb_get_config_descriptor(dev, config_idx, &config);
        if (r < LIBUSB_SUCCESS)
            return r;

        for (int iface_idx = 0; iface_idx < config->bNumInterfaces; iface_idx++)
        {
            const struct libusb_interface *iface = &config->interface[iface_idx];
            // Check all altsettings...
            for (int altsetting_idx = 0; altsetting_idx < iface->num_altsetting; altsetting_idx++)
            {
                const struct libusb_interface_descriptor *altsetting = &iface->altsetting[altsetting_idx];
                if (_libusb_interface_is_bluetooth(altsetting))
                {
                    *interface_number = altsetting->bInterfaceNumber;
                    return LIBUSB_SUCCESS;
                }
            }
        }
    }

    return LIBUSB_ERROR_NOT_FOUND;
}

int _libusb_dev_find_ep(libusb_device *dev, uint8_t *epnum, enum libusb_endpoint_direction dir, enum libusb_endpoint_transfer_type transfer_type)
{
    // Get currently active config...
    struct libusb_config_descriptor *config;
    int err = libusb_get_active_config_descriptor(dev, &config);
    if (err < LIBUSB_SUCCESS)
        return err;

    // Look for bluetooth interfaces...
    for (int iface_idx = 0; iface_idx < config->bNumInterfaces; iface_idx++)
    {
        const struct libusb_interface *iface = &config->interface[iface_idx];
        for (int altsetting_idx = 0; altsetting_idx < iface->num_altsetting; altsetting_idx++)
        {
            const struct libusb_interface_descriptor *altsetting = &iface->altsetting[altsetting_idx];
            if (_libusb_interface_is_bluetooth(altsetting))
            {
                for (int ep_idx = 0; ep_idx < altsetting->bNumEndpoints; ep_idx++)
                {
                    const struct libusb_endpoint_descriptor *ep_desc = &altsetting->endpoint[ep_idx];
                    if (_libusb_is_ep_dir(ep_desc, dir) && ep_desc->bmAttributes == transfer_type)
                    {
                        *epnum = ep_desc->bEndpointAddress;
                        return LIBUSB_SUCCESS;
                    }
                }
            }
        }
    }

    *epnum = 0;
    return LIBUSB_ERROR_NOT_FOUND;
}

int _libusb_dev_find_evt_ep(libusb_device *dev, uint8_t *epnum)
{
    return _libusb_dev_find_ep(dev, epnum, LIBUSB_ENDPOINT_IN, LIBUSB_ENDPOINT_TRANSFER_TYPE_INTERRUPT);
}

int _libusb_dev_find_acl_in_ep(libusb_device *dev, uint8_t *epnum)
{
    return _libusb_dev_find_ep(dev, epnum, LIBUSB_ENDPOINT_IN, LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK);
}

int _libusb_dev_find_acl_out_ep(libusb_device *dev, uint8_t *epnum)
{
    return _libusb_dev_find_ep(dev, epnum, LIBUSB_ENDPOINT_OUT, LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK);
}

int _ser_is_bluetooth_device(struct sp_port *dev, bool *is_bt)
{
    *is_bt = false;
    return LIBUSB_SUCCESS;
}
