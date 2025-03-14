using System.Runtime.InteropServices;

namespace UsbBluetooth
{
    internal static class UsbBluetoothWrapper
    {
        [DllImport("libusbbluetooth")]
        internal static extern UsbBluetoothStatus usbbluetooth_init();

        [DllImport("libusbbluetooth")]
        internal static extern void usbbluetooth_exit();

        internal unsafe struct UsbBluetoothDeviceStruct
        {
            byte ref_count;
            int type;
            void* device;
            void* context;
        }

        [DllImport("libusbbluetooth")]
        internal unsafe static extern UsbBluetoothStatus usbbluetooth_get_device_list(UsbBluetoothDeviceStruct*** list);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern void usbbluetooth_free_device_list(UsbBluetoothDeviceStruct*** list);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern UsbBluetoothDeviceStruct* usbbluetooth_reference_device(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern void usbbluetooth_unreference_device(UsbBluetoothDeviceStruct** dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern void usbbluetooth_device_vid_pid(UsbBluetoothDeviceStruct* dev, ushort *vid, ushort *pid);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern IntPtr usbbluetooth_device_manufacturer(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern IntPtr usbbluetooth_device_product(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern IntPtr usbbluetooth_device_serial_num(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern IntPtr usbbluetooth_device_description(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern UsbBluetoothStatus usbbluetooth_open(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern void usbbluetooth_close(UsbBluetoothDeviceStruct* dev);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern UsbBluetoothStatus usbbluetooth_write(UsbBluetoothDeviceStruct* dev, byte* data, ushort size);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern UsbBluetoothStatus usbbluetooth_read(UsbBluetoothDeviceStruct* dev, byte* data, ushort* size);

        [DllImport("libusbbluetooth")]
        internal unsafe static extern void usbbluetooth_log_set_level(UsbBluetoothLogLevel level);
    }
}
