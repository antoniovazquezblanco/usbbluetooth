#!/usr/bin/env python

import sys
import ctypes
from enum import IntEnum
from importlib.resources import files


def _lib_name() -> str:
    '''
    Returns an string with the expected library name for the current platform.
    '''
    lib_names = {
        'darwin': 'libusbbluetooth.dylib',
        'win32': 'libusbbluetooth.dll',
        'linux': 'libusbbluetooth.so'
    }
    return lib_names.get(sys.platform, 'libusbbluetooth.so')


def _load_lib():
    '''
    Attempt to load a library from package resources.
    '''
    library_path = files('usbbluetooth.lib').joinpath(_lib_name())
    try:
        return ctypes.cdll.LoadLibrary(str(library_path))
    except OSError as e:
        print(e)
        return None


# Attempt to load the corresponding C library...
_lib = _load_lib()
if _lib is None:
    raise ImportError(
        "ERROR: Failed to load the dynamic library for usbbluetooth.")


class _CStatus(IntEnum):
    OK = 0
    ERR_UNK = -1,
    ERR_NOMEM = -2,
    ERR_DEVICE_CLOSED = -3,


class _CDevice(ctypes.Structure):
    _fields_ = (("ref_count", ctypes.c_uint8),
                ("type", ctypes.c_int),
                ("device", ctypes.c_voidp),
                ("context", ctypes.c_voidp))


def _setup_func(lib, fname, restype, argtypes) -> None:
    '''
    Setup function type info. Export as global.
    '''
    try:
        method = getattr(lib, fname)
        method.restype = restype
        method.argtypes = argtypes
        globals()[fname] = method
    except AttributeError:
        raise ImportError(
            f"ERROR: Fail to setup '{fname}' function prototype. Make sure you are loading the correct library version.")


# Actual function prototypes...
_setup_func(_lib, "usbbluetooth_init", _CStatus, None)
_setup_func(_lib, "usbbluetooth_exit", None, None)
_setup_func(_lib, "usbbluetooth_get_device_list", _CStatus,
            [ctypes.POINTER(ctypes.POINTER(ctypes.POINTER(_CDevice)))])
_setup_func(_lib, "usbbluetooth_free_device_list", None,
            [ctypes.POINTER(ctypes.POINTER(ctypes.POINTER(_CDevice)))])
_setup_func(_lib, "usbbluetooth_reference_device", ctypes.POINTER(_CDevice),
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_unreference_device", None,
            [ctypes.POINTER(ctypes.POINTER(_CDevice))])
_setup_func(_lib, "usbbluetooth_device_vid_pid", None,
            [ctypes.POINTER(_CDevice), ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)])
_setup_func(_lib, "usbbluetooth_device_manufacturer", ctypes.c_char_p,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_device_product", ctypes.c_char_p,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_device_serial_num", ctypes.c_char_p,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_device_description", ctypes.c_char_p,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_open", _CStatus,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_close", None,
            [ctypes.POINTER(_CDevice)])
_setup_func(_lib, "usbbluetooth_write", _CStatus,
            [ctypes.POINTER(_CDevice), ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16])
_setup_func(_lib, "usbbluetooth_read", _CStatus,
            [ctypes.POINTER(_CDevice), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16)])
_setup_func(_lib, "usbbluetooth_log_set_level", None, [ctypes.c_int])
