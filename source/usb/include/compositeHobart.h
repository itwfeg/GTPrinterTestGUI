#ifndef COMPOSITEHOBART_H
#define COMPOSITEHOBART_H
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "usbPrinter.h"
#include "usbWeigher.h"
#include "usb_device_descriptor.h"

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#endif

#if defined(__GIC_PRIO_BITS)
#define USB_DEVICE_INTERRUPT_PRIORITY (25U)
#elif defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS >= 3)
#define USB_DEVICE_INTERRUPT_PRIORITY (6U)
#else
#define USB_DEVICE_INTERRUPT_PRIORITY (3U)
#endif


typedef struct _usb_device_composite_struct
{
    usb_device_handle                   deviceHandle;
    usb_device_printer_t                printer;
    usb_device_weigher_t                weigher;
    TaskHandle_t                        applicationTaskHandle;
    TaskHandle_t                        deviceTaskHandle;
    uint8_t                             speed;
    uint8_t                             attach;
    uint8_t                             currentConfiguration;
    uint8_t                             currentInterfaceAlternateSetting[ USB_COMPOSITE_INTERFACE_COUNT ];
} usb_device_composite_struct_t;
#endif