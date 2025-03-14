using System;
using System.Runtime.InteropServices;
using static UsbBluetooth.UsbBluetoothWrapper;

namespace UsbBluetooth
{
    public class UsbBluetoothDevice : IDisposable
    {
        private const int BUFFER_SIZE = 1024;
        private unsafe UsbBluetoothDeviceStruct* m_BluetoothDeviceStruct;

        internal unsafe UsbBluetoothDevice(UsbBluetoothDeviceStruct* s)
        {
            m_BluetoothDeviceStruct = usbbluetooth_reference_device(s);
        }

        ~UsbBluetoothDevice()
        {
            Dispose();
        }

        public unsafe void Dispose()
        {
            fixed (UsbBluetoothDeviceStruct** ptr = &m_BluetoothDeviceStruct)
            {
                usbbluetooth_unreference_device(ptr);
            }
        }

        public unsafe UsbBluetoothStatus Open()
        {
            UsbBluetoothStatus status = usbbluetooth_open(m_BluetoothDeviceStruct);
            status.RaiseExceptions();
            return status;
        }

        public unsafe void Close()
        {
            usbbluetooth_close(m_BluetoothDeviceStruct);
        }

        public unsafe byte[] Read()
        {
            byte[] buffer = new byte[BUFFER_SIZE];
            ushort buffer_len = BUFFER_SIZE;
            fixed (byte* buffptr = buffer)
            {
                UsbBluetoothStatus status = usbbluetooth_read(m_BluetoothDeviceStruct, buffptr, &buffer_len);
                status.RaiseExceptions();
            }
            return buffer[0..buffer_len];
        }

        public unsafe void Write(byte[] data)
        {
            fixed (byte* dataptr = data)
            {
                UsbBluetoothStatus status = usbbluetooth_write(m_BluetoothDeviceStruct, dataptr, (ushort)data.Length);
                status.RaiseExceptions();
            }
        }

        public unsafe void GetVidPid(ushort *vid, ushort *pid)
        {
            usbbluetooth_device_vid_pid(m_BluetoothDeviceStruct, vid, pid);
        }

        public unsafe String GetManufacturer()
        {
            IntPtr manuf = usbbluetooth_device_manufacturer(m_BluetoothDeviceStruct);
            return Marshal.PtrToStringAnsi(manuf);
        }

        public unsafe string GetProduct()
        {
            IntPtr prod = usbbluetooth_device_product(m_BluetoothDeviceStruct);
            return Marshal.PtrToStringAnsi(prod);
        }

        public unsafe string GetSerialNumber()
        {
            IntPtr sn = usbbluetooth_device_serial_num(m_BluetoothDeviceStruct);
            return Marshal.PtrToStringAnsi(sn);
        }

        public unsafe string GetDescription()
        {
            IntPtr desc = usbbluetooth_device_description(m_BluetoothDeviceStruct);
            return Marshal.PtrToStringAnsi(desc);
        }

        public unsafe override string ToString()
        {
            return "UsbBluetoothDevice[" + GetDescription() + "]";
        }
    }
}
