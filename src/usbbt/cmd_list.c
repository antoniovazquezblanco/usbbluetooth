#include "cmd_list.h"

#include <usbbluetooth.h>
#include <stdio.h>
#include <stdlib.h>

static void print_dev(usbbluetooth_device_t *dev)
{
    usbbluetooth_open(dev);
    char *desc = usbbluetooth_device_description(dev);
    printf("Device %s @ 0x%p\n", desc, dev);
    free(desc);
    usbbluetooth_close(dev);
}

static void print_devs(usbbluetooth_device_t **devs)
{
    usbbluetooth_device_t *dev;
    for (int i = 0; (dev = devs[i]) != NULL; i++)
        print_dev(dev);
}

int cmd_list()
{
    int ret = 0;
    usbbluetooth_device_t **devs = NULL;

    usbbluetooth_status_t r = usbbluetooth_init();
    if (r < 0)
    {
        fprintf(stderr, "Could not initialize usbbluetooth (r=%d, s=%s)\n", (int)r, usbbluetooth_status_name(r));
        ret = -1;
        goto exit;
    }

    r = usbbluetooth_get_device_list(&devs);
    if (r < 0)
    {
        fprintf(stderr, "Could get a device list (r=%d, s=%s)\n", (int)r, usbbluetooth_status_name(r));
        ret = -1;
        goto exit;
    }

    printf("Listing devices:\n");
    print_devs(devs);

exit:
    usbbluetooth_free_device_list(&devs);
    usbbluetooth_exit();

    return ret;
}