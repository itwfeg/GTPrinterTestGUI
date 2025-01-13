#ifndef USBHOBARTPRINTER_H
#define USBHOBARTPRINTER_H



extern usb_status_t USB_DevicePrinterInit(usb_device_composite_struct_t *deviceComposite);
extern usb_status_t USB_DevicePrinterCallback(class_handle_t handle, uint32_t event, void *param);
extern usb_status_t USB_DevicePrinterSetConfigure(class_handle_t handle, uint8_t configure);
extern usb_status_t USB_DevicePrinterSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting);
#endif 