/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USB_DEVICE_SPECIFIC_BCD_VERSION (0x0200U)
#define USB_DEVICE_DEMO_BCD_VERSION (0x0101U)

#define USB_DEVICE_VID (0x128CU)
#define USB_DEVICE_PID (0x2000U)

#define USB_DEVICE_CLASS (0x00U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)

#define USB_DEVICE_MAX_POWER (0x32U)

#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL (sizeof(g_UsbDeviceConfigurationDescriptor))
#define USB_DESCRIPTOR_LENGTH_STRING0 (sizeof(g_UsbDeviceString0))
#define USB_DESCRIPTOR_LENGTH_STRING1 (sizeof(g_UsbDeviceString1))
#define USB_DESCRIPTOR_LENGTH_STRING2 (sizeof(g_UsbDeviceString2))

#define USB_DEVICE_CONFIGURATION_COUNT (1U)
#define USB_DEVICE_STRING_COUNT (3U)
#define USB_DEVICE_LANGUAGE_COUNT (1U)

#define USB_COMPOSITE_CONFIGURE_INDEX (1U)

/* #define USB_PRINTER_CONFIGURE_INDEX (1U) */
#define USB_BOOTLOADER_INTERFACE_COUNT (3U)

#define USB_BOOTLOADER_INTERFACE_INDEX (0U)
#define USB_BOOTLOADER_INTERFACE_ALTERNATE_COUNT (1U)
#define USB_BOOTLOADER_INTERFACE_ALTERNATE_0 (0U)
#define USB_BOOTLOADER_ENDPOINT_COUNT (4U)   /* was 4 */  
#define USB_BOOTLOADER_INTERRUPT_ENDPOINT_OUT (1U)
#define USB_BOOTLOADER_INTERRUPT_ENDPOINT_IN (2U)
#define USB_BOOTLOADER_BULK_ENDPOINT_OUT (3U)
#define USB_BOOTLOADER_BULK_ENDPOINT_IN (4U)

#define USB_BOOTLOADER_CLASS (0x00U)
#define USB_BOOTLOADER_SUBCLASS (0x00U)
#define USB_BOOTLOADER_PROTOCOL (0x00U)

#define HS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define HS_BOOTLOADER_INTERRUPT_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_BOOTLOADER_INTERRUPT_OUT_INTERVAL (0x04U)
#define HS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE (64U)
#define FS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE (64U)
#define HS_BOOTLOADER_INTERRUPT_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_BOOTLOADER_INTERRUPT_IN_INTERVAL (0x04U)

#define HS_BOOTLOADER_BULK_OUT_PACKET_SIZE (512U)  
#define FS_BOOTLOADER_BULK_OUT_PACKET_SIZE (512U)  
#define HS_BOOTLOADER_BULK_IN_PACKET_SIZE (512U)   
#define FS_BOOTLOADER_BULK_IN_PACKET_SIZE (512U)   
#define HS_BOOTLOADER_BULK_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_BOOTLOADER_BULK_OUT_INTERVAL (0x04U)
#define HS_BOOTLOADER_BULK_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_BOOTLOADER_BULK_IN_INTERVAL (0x04U)

#define USB_WEIGHER_CLASS (0x00U)
#define USB_WEIGHER_SUBCLASS (0x00U)
#define USB_WEIGHER_PROTOCOL (0x00U)

#define USB_WEIGHER_INTERFACE_COUNT (2U)
#define USB_WEIGHER_INTERFACE_INDEX (1U)
#define USB_WEIGHER_INTERFACE_ALTERNATE_COUNT (1U)
#define USB_WEIGHER_INTERFACE_ALTERNATE_0 (0U)
#define USB_WEIGHER_IN_BUFFER_LENGTH (8U)
#define USB_WEIGHER_ENDPOINT_COUNT (2U)
#define USB_WEIGHER_INTERRUPT_ENDPOINT_OUT (5U)
#define USB_WEIGHER_INTERRUPT_ENDPOINT_IN (6U)

#define HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define HS_WEIGHER_INTERRUPT_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_WEIGHER_INTERRUPT_OUT_INTERVAL (0x04U)
#define HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE (64U)
#define FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE (64U)
#define HS_WEIGHER_INTERRUPT_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_WEIGHER_INTERRUPT_IN_INTERVAL (0x04U)

#define USB_COMPOSITE_INTERFACE_COUNT (USB_BOOTLOADER_INTERFACE_COUNT)
/*******************************************************************************
 * API
 ******************************************************************************/

/* Configure the device according to the USB speed. */
extern usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed);

/* Get device descriptor request */
usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                           usb_device_get_device_descriptor_struct_t *deviceDescriptor);

/* Get device configuration descriptor request */
usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor);

/* Get device string descriptor request */
usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                           usb_device_get_string_descriptor_struct_t *stringDescriptor);

#endif /* __USB_DEVICE_DESCRIPTOR_H__ */
