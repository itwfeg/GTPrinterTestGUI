#ifndef USBHOBARTWEIGHER_H
#define USBHOBARTWEIGHER_H

extern usb_status_t USB_DeviceWeigherInit(usb_device_composite_struct_t *deviceComposite);
extern usb_status_t USB_DeviceWeigherCallback(class_handle_t handle, uint32_t event, void *param);
extern usb_status_t USB_DeviceWeigherSetConfigure(class_handle_t handle, uint8_t configure);
extern usb_status_t USB_DeviceWeigherInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting);

#endif