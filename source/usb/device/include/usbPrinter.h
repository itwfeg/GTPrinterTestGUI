#ifndef USBPRINTER_H
#define USBPRINTER_H

/*! @brief The class code of the HID class */
#define USB_DEVICE_CONFIG_HOBART_CLASS_CODE (0x00U)

typedef enum _usb_device_printer_event
{
    kUSB_DevicePrinterEventSendResponse = 0x01U, /*!< Send data completed or cancelled etc*/
    kUSB_DevicePrinterEventRecvResponse,         /*!< Data received or cancelled etc*/
    kUSB_DevicePrinterEventRecvBulkResponse,
    //kUSB_DevicePrinterEventGetReport,            /*!< Get report request */
    //kUSB_DevicePrinterEventGetIdle,              /*!< Get idle request */
    //kUSB_DevicePrinterEventGetProtocol,          /*!< Get protocol request */
    //kUSB_DevicePrinterEventSetReport,            /*!< Set report request */
    //kUSB_DevicePrinterEventSetIdle,              /*!< Set idle request */
    kUSB_DevicePrinterEventSetProtocol,          /*!< Set protocol request */
    //kUSB_DevicePrinterEventRequestReportBuffer,  /*!< Get buffer to save the data of the set report request. */
} usb_device_printer_event_t;

/* class header for hobart pritner */
typedef struct _usb_device_printer
{
    usb_device_handle   deviceHandle;
    usb_device_class_config_struct_t *configStruct; /*!< The configuration of the class. */
    usb_device_interface_struct_t *interfaceHandle; /*!< Current interface handle */
    uint8_t attach;
    uint32_t msgLength;
    uint8_t configuration;                          /*!< Current configuration */
    uint8_t interfaceNumber;                        /*!< The interface number of the class */
    uint8_t alternate;                              /*!< Current alternate setting of the interface */
    uint8_t *interruptInPipeDataBuffer;             /*!< IN pipe data buffer backup when stall */
    uint32_t interruptInPipeDataLen;                /*!< IN pipe data length backup when stall  */
    uint8_t interruptOutEp;                         /*!< Interrupt out pipe address */
    uint8_t *interruptOutPipeDataBuffer;            /*!< OUT pipe data buffer backup when stall */
    uint32_t interruptOutPipeDataLen;               /*!< OUT pipe data length backup when stall  */
    uint8_t bulkOutEp;                              /*!< Bulk out pipe address */
    uint8_t *bulkOutPipeDataBuffer;                 /*!< OUT pipe data buffer backup when stall */
    uint32_t bulkOutPipeDataLen;                    /*!< OUT pipe data length backup when stall  */    
    uint8_t interruptInPipeBusy;                    /*!< Interrupt IN pipe busy flag */
    uint8_t interruptOutPipeBusy;                   /*!< Interrupt OUT pipe busy flag */
    uint8_t interruptInPipeStall;                   /*!< Interrupt IN pipe stall flag */
    uint8_t interruptOutPipeStall;                  /*!< Interrupt OUT pipe stall flag */   
    uint8_t bulkOutPipeBusy;                        /*!< Bulk OUT pipe busy flag */
    uint8_t bulkOutPipeStall;
} usb_device_printer_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the Hobart Printer class.
 *
 * This function is used to initialize the HID class. This function only can be called by #USB_DeviceClassInit.
 *
 * @param[in] controllerId   The controller ID of the USB IP. See the enumeration #usb_controller_index_t.
 * @param[in] config          The class configuration information.
 * @param[out] handle          An parameter used to return pointer of the HID class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceHBPrinterInit(uint8_t controllerId,
                                      usb_device_class_config_struct_t *config,
                                      class_handle_t *handle);

/*!
 * @brief Deinitializes the device Hobart Printer class.
 *
 * The function deinitializes the device Hobart Printer class. This function only can be called by #USB_DeviceClassDeinit.
 *
 * @param[in] handle The Hobart Printer class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceHBPrinterDeinit(class_handle_t handle);

/*!
 * @brief Handles the event passed to the Hobart Printer class.
 *
 * This function handles the event passed to the Hobart Printer class. This function only can be called by #USB_DeviceClassEvent.
 *
 * @param[in] handle          The Hobart Printer class handle received from the usb_device_class_config_struct_t::classHandle.
 * @param[in] event           The event codes. See the enumeration usb_device_class_event_t.
 * @param[in,out] param           The parameter type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe is stalled by the caller.
 */
extern usb_status_t USB_DeviceHBPrinterEvent(void *handle, uint32_t event, void *param);

/*!
 * @name USB device Hobart Printer class APIs
 * @{
 */

/*!
 * @brief Sends data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 * The function calls #USB_DeviceSendRequest internally.
 *
 * @param[in] handle The Hobart Printer class handle received from usb_device_class_config_struct_t::classHandle.
 * @param[in] ep     Endpoint index.
 * @param[in] buffer The memory address to hold the data need to be sent.
 * @param[in] length The data length to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The function can only be called in the same context.
 *
 * @note The return value indicates whether the sending request is successful or not. The transfer done is notified by
 * usb_device_hid_interrupt_in.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is received through the
 * endpoint
 * callback).
 */
extern usb_status_t USB_DeviceHBPrinterSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 * The function calls #USB_DeviceRecvRequest internally.
 *
 * @param[in] handle The Hobart Printer class handle received from the usb_device_class_config_struct_t::classHandle.
 * @param[in] ep     Endpoint index.
 * @param[in] buffer The memory address to save the received data.
 * @param[in] length The data length to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The function can only be called in the same context.
 *
 * @note The return value indicates whether the receiving request is successful or not. The transfer done is notified by
 * usb_device_hid_interrupt_out.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is received through the
 * endpoint
 * callback).
 */
extern usb_status_t USB_DeviceHBPrinterRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif