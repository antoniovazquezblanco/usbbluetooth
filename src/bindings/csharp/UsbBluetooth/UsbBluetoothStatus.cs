namespace UsbBluetooth
{
    public enum UsbBluetoothStatus
    {
        OK = 0,
        ERR_UNK = -1,
        ERR_NO_MEM = -2,
        ERR_DEVICE_CLOSED = -3,
    }

    class UnknownException : Exception
    {
    }

    class DeviceClosedException : Exception
    {
    }

    public static class Extensions
    {
        public static void RaiseExceptions(this UsbBluetoothStatus status)
        {
            switch (status)
            {
                case UsbBluetoothStatus.OK:
                    return;
                case UsbBluetoothStatus.ERR_UNK:
                    throw new UnknownException();
                case UsbBluetoothStatus.ERR_NO_MEM:
                    throw new OutOfMemoryException();
                case UsbBluetoothStatus.ERR_DEVICE_CLOSED:
                    throw new DeviceClosedException();
                default:
                    throw new UnknownException();
            }
        }
    }
}
