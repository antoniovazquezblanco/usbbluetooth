#include <usbbluetooth_log.h>

#include <libusb.h>
#include <stdarg.h>
#include <stdio.h>

static usbbluetooth_log_level_t _log_level = USBBLUETOOTH_LOG_NONE;

static char _log_tag(usbbluetooth_log_level_t l)
{
    switch (l)
    {
    case USBBLUETOOTH_LOG_NONE:
        return ' ';
    case USBBLUETOOTH_LOG_ERROR:
        return 'E';
    case USBBLUETOOTH_LOG_WARN:
        return 'W';
    case USBBLUETOOTH_LOG_INFO:
        return 'I';
    case USBBLUETOOTH_LOG_DEBUG:
        return 'D';
    }
    return ' ';
}

static enum libusb_log_level _log_level_to_libusb(usbbluetooth_log_level_t l)
{
    switch (l)
    {
    case USBBLUETOOTH_LOG_NONE:
        return LIBUSB_LOG_LEVEL_NONE;
    case USBBLUETOOTH_LOG_ERROR:
        return LIBUSB_LOG_LEVEL_ERROR;
    case USBBLUETOOTH_LOG_WARN:
        return LIBUSB_LOG_LEVEL_WARNING;
    case USBBLUETOOTH_LOG_INFO:
        return LIBUSB_LOG_LEVEL_INFO;
    case USBBLUETOOTH_LOG_DEBUG:
        return LIBUSB_LOG_LEVEL_DEBUG;
    default:
        return LIBUSB_LOG_LEVEL_NONE;
    }
}

static void _log_format(usbbluetooth_log_level_t l, const char *fmt, va_list args)
{
    printf("[%c]", _log_tag(l));
    vprintf(fmt, args);
    printf("\n");
}

void USBBLUETOOTH_CALL usbbluetooth_log_set_level(usbbluetooth_log_level_t level)
{
    _log_level = level;
    libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, _log_level_to_libusb(level));
    usbbluetooth_log_debug("usbbluetooth_log_set_level[l=%d]", level);
}

void usbbluetooth_log(usbbluetooth_log_level_t level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    if (level <= _log_level)
        _log_format(level, fmt, args);
    va_end(args);
}
