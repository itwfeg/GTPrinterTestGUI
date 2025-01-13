#ifndef USBHOBARTBOOTLOADER_H
#define USBHOBARTBOOTLOADER_H

extern usb_status_t USB_DeviceBootloaderInit(usb_device_composite_struct_t *deviceComposite);
extern usb_status_t USB_DeviceBootloaderCallback(class_handle_t handle, uint32_t event, void *param);
extern usb_status_t USB_DeviceBootloaderSetConfigure(class_handle_t handle, uint8_t configure);
extern usb_status_t USB_DeviceBootloaderSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting);

#endif 