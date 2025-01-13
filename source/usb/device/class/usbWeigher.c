#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"

#if ((defined(USB_DEVICE_CONFIG_HOBART_WEIGHER)) && (USB_DEVICE_CONFIG_HOBART_WEIGHER > 0U))
#include "usbWeigher.h"

static usb_status_t USB_DeviceWeigherAllocateHandle( usb_device_weigher_t **pHandle );
static usb_status_t USB_DeviceWeigherFreeHandle( usb_device_weigher_t *pHandle );
static usb_status_t USB_DeviceWeigherInterruptIn( usb_device_handle handle,
                                                  usb_device_endpoint_callback_message_struct_t *message,
                                                  void *callbackParam );
static usb_status_t USB_DeviceWeigherInterruptOut( usb_device_handle handle,
                                                   usb_device_endpoint_callback_message_struct_t *message,
                                                   void *callbackParam );
static usb_status_t USB_DeviceWeigherEndpointsInit( usb_device_weigher_t *pHandle );
static usb_status_t USB_DeviceWeigherEndpointsDeinit( usb_device_weigher_t *pHandle );

USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_weigher_t
    s_UsbDeviceWeigherHandle[USB_DEVICE_CONFIG_HOBART];


/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherAllocateHandle( usb_device_printer_t **pHandle )

      \brief
        This function allocates a device hobart weigher class handle.

      \param pHandle    is out parameter, is used to return pointer of the hobart
                        weigher class handle to the caller.
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherAllocateHandle( usb_device_weigher_t **pHandle )
{
    uint32_t count;
    for (count = 0U; count < USB_DEVICE_CONFIG_HOBART; count++)
    {
        if (NULL == s_UsbDeviceWeigherHandle[count].deviceHandle)
        {
            *pHandle = &s_UsbDeviceWeigherHandle[count];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}


/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherFreeHandle( usb_device_printer_t *pHandle )

      \brief
        This function frees a hobart weigher class handle.

      \param pHandle    hobart weigher class handle.
                         
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherFreeHandle( usb_device_weigher_t *pHandle )
{
    pHandle->deviceHandle        = NULL;
    pHandle->configStruct  = (usb_device_class_config_struct_t *)NULL;
    pHandle->configuration = 0U;
    pHandle->alternate     = 0U;
    return kStatus_USB_Success;
}


/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherInterruptIn( usb_device_handle handle,
                                                            usb_device_endpoint_callback_message_struct_t *message,
                                                            void *callbackParam )

      \brief
        This function is the weigher interrupt in endpoint callback function

      \param handle          The device handle. It equals the value returned from USB_DeviceInit.
      \param message         The result of the interrupt IN pipe transfer.    
      \param callbackParam  The parameter for this callback. It is same with
                            usb_device_endpoint_callback_struct_t::callbackParam. 
                            In the class, the value is the weigher class handle.
                         
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherInterruptIn( usb_device_handle handle,
                                                  usb_device_endpoint_callback_message_struct_t *message,
                                                  void *callbackParam )
{
    usb_device_weigher_t *pPHandle;
    usb_status_t status = kStatus_USB_Error;

    /* get the hobart weigher class handle */
    pPHandle = (usb_device_weigher_t *)callbackParam;

    if (NULL == pPHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    pPHandle->interruptInPipeBusy = 0U;
    if ((NULL != pPHandle->configStruct) && (NULL != pPHandle->configStruct->classCallback))
    {
        /* notify the application data sent by calling the hid class callback. classCallback is initialized
           in classInit of s_UsbDeviceClassInterfaceMap,it is from the second parameter of classInit */
        status =
            pPHandle->configStruct->classCallback((class_handle_t)pPHandle, kUSB_DeviceWeigherEventSendResponse, message);
    }

    return status;
}

/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherInterruptOut( usb_device_handle handle,
                                                             usb_device_endpoint_callback_message_struct_t *message,
                                                             void *callbackParam )

      \brief
        This function is the weigher interrupt out endpoint callback function

      \param handle          The device handle. It equals the value returned from USB_DeviceInit.
      \param message         The result of the interrupt OUT pipe transfer.    
      \param callbackParam   The parameter for this callback. It is same with
                             usb_device_endpoint_callback_struct_t::callbackParam. 
                             In the class, the value is the weigher class handle.
                         
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherInterruptOut( usb_device_handle handle,
                                                   usb_device_endpoint_callback_message_struct_t *message,
                                                   void *callbackParam )
{
    usb_device_weigher_t *pHandle;
    usb_status_t status = kStatus_USB_Error;

    /* get the hobart weigher class handle */
    pHandle = (usb_device_weigher_t *)callbackParam;

    if (NULL == pHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    pHandle->interruptOutPipeBusy = 0U;
    if ((NULL != pHandle->configStruct) && (NULL != pHandle->configStruct->classCallback))
    {
        /* notify the application data sent by calling the hobart class callback. classCallback is initialized
           in classInit of s_UsbDeviceClassInterfaceMap,it is from the second parameter of classInit */
        status =
            pHandle->configStruct->classCallback((class_handle_t)pHandle, kUSB_DeviceWeigherEventRecvResponse, message);
    }

    return status;
}

/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherEndpointsInit( usb_device_printer_t *pHandle )

      \brief
        This function initializes the endpoints of the hobart class

      \param pHandle  The hobart weigher class handle. It equals the value 
                      returned from usb_device_class_config_struct_t::classHandle.
                          
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherEndpointsInit(usb_device_weigher_t *pHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t status                      = kStatus_USB_Error;
    uint32_t count;
    uint32_t index;

    /* Check the configuration is valid or not. */
    if (0U == pHandle->configuration)
    {
        return status;
    }

    if (pHandle->configuration > pHandle->configStruct->classInfomation->configurations)
    {
        return status;
    }

    /* Get the interface list of the new configuration. */
    if (NULL == pHandle->configStruct->classInfomation->interfaceList)
    {
        return status;
    }
    interfaceList = &pHandle->configStruct->classInfomation->interfaceList[pHandle->configuration - 1U];

    /* Find interface by using the alternate setting of the interface. */
    for (count = 0U; count < interfaceList->count; count++)
    {
        if ( 0x00U == interfaceList->interfaces[count].classCode )
        {
            for (index = 0U; index < interfaceList->interfaces[count].count; index++)
            {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == pHandle->alternate)
                {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            pHandle->interfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface)
    {
        /* Return error if the interface is not found. */
        return status;
    }

    /* Keep new interface handle. */
    pHandle->interfaceHandle = interface;

    /* Initialize the endpoints of the new interface. */
    for (count = 0U; count < interface->endpointList.count; count++)
    {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0U;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                       USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
        {
            epCallback.callbackFn                = USB_DeviceWeigherInterruptIn;
            pHandle->interruptInPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
            pHandle->interruptInPipeStall      = 0U;
            pHandle->interruptInPipeDataLen    = 0U;
        }
        else
        {
            epCallback.callbackFn                 = USB_DeviceWeigherInterruptOut;
            pHandle->interruptOutPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
            pHandle->interruptOutPipeStall      = 0U;
            pHandle->interruptOutPipeDataLen    = 0U;
        }
        epCallback.callbackParam = pHandle;

        status = USB_DeviceInitEndpoint(pHandle->deviceHandle, &epInitStruct, &epCallback);
    }
    return status;
}

/******************************************************************************/
/*!   \fn static usb_status_t USB_DeviceWeigherEndpointsDeinit( usb_device_printer_t *pHandle )

      \brief
        This function de-initializes the endpoints of the hobart weigher class.

      \param pHandle  The hobart weigher class handle. It equals the value 
                      returned from usb_device_class_config_struct_t::classHandle.
                          
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceWeigherEndpointsDeinit(usb_device_weigher_t *pHandle)
{
    usb_status_t status = kStatus_USB_Error;
    uint32_t count;

    if (NULL == pHandle->interfaceHandle)
    {
        return status;
    }
    /* De-initialize all endpoints of the interface */
    for (count = 0U; count < pHandle->interfaceHandle->endpointList.count; count++)
    {
        status = USB_DeviceDeinitEndpoint(pHandle->deviceHandle,
                                          pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress);
    }
    pHandle->interfaceHandle = NULL;
    return status;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceHBWeigherEvent( void *handle, uint32_t event, void *param )

      \brief
        This function handles the event passed to the hobart weigher class.

      \param pHandle  The hobart weigher class handle. It equals the value 
                      returned from usb_device_class_config_struct_t::classHandle.
      \param event    The event code. 
                      Please refer to the enumeration usb_device_class_event_t.
                          
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceHBWeigherEvent(void *handle, uint32_t event, void *param)
{
    usb_device_weigher_t *pHandle;
    usb_status_t error = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint32_t count;
    uint8_t *temp8;
    uint8_t alternate;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;

    if ((NULL == param) || (NULL == handle))
    {
        return kStatus_USB_InvalidHandle;
    }

    /* Get the hid class handle. */
    pHandle = (usb_device_weigher_t *)handle;

    switch (eventCode)
    {
        case kUSB_DeviceClassEventDeviceReset:
            /* Bus reset, clear the configuration. */
            pHandle->configuration        = 0U;
            pHandle->interruptInPipeBusy  = 0U;
            pHandle->interruptOutPipeBusy = 0U;
            pHandle->interfaceHandle      = NULL;
            error                           = kStatus_USB_Success;
            break;
        case kUSB_DeviceClassEventSetConfiguration:
            /* Get the new configuration. */
            temp8 = ((uint8_t *)param);
            if (NULL == pHandle->configStruct)
            {
                break;
            }
            if (*temp8 == pHandle->configuration)
            {
                error = kStatus_USB_Success;
                break;
            }

            /* De-initialize the endpoints when current configuration is none zero. */
            if (0U != pHandle->configuration)
            {
                error = USB_DeviceWeigherEndpointsDeinit(pHandle);
            }
            /* Save new configuration. */
            pHandle->configuration = *temp8;
            /* Clear the alternate setting value. */
            pHandle->alternate = 0U;

            /* Initialize the endpoints of the new current configuration by using the alternate setting 0. */
            error = USB_DeviceWeigherEndpointsInit(pHandle);
            break;
        case kUSB_DeviceClassEventSetInterface:
            if (NULL == pHandle->configStruct)
            {
                break;
            }
            /* Get the new alternate setting of the interface */
            interfaceAlternate = *((uint16_t *)param);
            /* Get the alternate setting value */
            alternate = (uint8_t)(interfaceAlternate & 0xFFU);

            /* Whether the interface belongs to the class. */
            if (pHandle->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8U)))
            {
                break;
            }
            /* Only handle new alternate setting. */
            if (alternate == pHandle->alternate)
            {
                error = kStatus_USB_Success;
                break;
            }
            /* De-initialize old endpoints */
            error                = USB_DeviceWeigherEndpointsDeinit(pHandle);
            pHandle->alternate = alternate;
            /* Initialize new endpoints */
            error = USB_DeviceWeigherEndpointsInit(pHandle);
            break;
        case kUSB_DeviceClassEventSetEndpointHalt:
            if ((NULL == pHandle->configStruct) || (NULL == pHandle->interfaceHandle))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            for (count = 0U; count < pHandle->interfaceHandle->endpointList.count; count++)
            {
                if (*temp8 == pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
                {
                    /* Only stall the endpoint belongs to the class */
                    if (USB_IN == ((pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                                   USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
                    {
                        pHandle->interruptInPipeStall = 1U;
                    }
                    else
                    {
                        pHandle->interruptOutPipeStall = 1U;
                    }
                    error = USB_DeviceStallEndpoint(pHandle->deviceHandle, *temp8);
                }
            }
            break;
        case kUSB_DeviceClassEventClearEndpointHalt:
            if ((NULL == pHandle->configStruct) || (NULL == pHandle->interfaceHandle))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            for (count = 0U; count < pHandle->interfaceHandle->endpointList.count; count++)
            {
                if (*temp8 == pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
                {
                    /* Only un-stall the endpoint belongs to the class */
                    error = USB_DeviceUnstallEndpoint(pHandle->deviceHandle, *temp8);
                    if (USB_IN == (((*temp8) & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                                   USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
                    {
                        if (0U != pHandle->interruptInPipeStall)
                        {
                            pHandle->interruptInPipeStall = 0U;
                            if ((uint8_t *)USB_INVALID_TRANSFER_BUFFER != pHandle->interruptInPipeDataBuffer)
                            {
                                error = USB_DeviceSendRequest(
                                    pHandle->deviceHandle,
                                    (pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK),
                                    pHandle->interruptInPipeDataBuffer, pHandle->interruptInPipeDataLen);
                                if (kStatus_USB_Success != error)
                                {
                                    usb_device_endpoint_callback_message_struct_t endpointCallbackMessage;
                                    endpointCallbackMessage.buffer  = pHandle->interruptInPipeDataBuffer;
                                    endpointCallbackMessage.length  = pHandle->interruptInPipeDataLen;
                                    endpointCallbackMessage.isSetup = 0U;
                                    (void)USB_DeviceWeigherInterruptIn(pHandle->deviceHandle, (void *)&endpointCallbackMessage,
                                                                   handle);
                                }
                                pHandle->interruptInPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
                                pHandle->interruptInPipeDataLen    = 0U;
                            }
                        }
                    }
                    else
                    {
                        if (0U != pHandle->interruptOutPipeStall)
                        {
                            pHandle->interruptOutPipeStall = 0U;
                            if ((uint8_t *)USB_INVALID_TRANSFER_BUFFER != pHandle->interruptOutPipeDataBuffer)
                            {
                                error = USB_DeviceRecvRequest(
                                    pHandle->deviceHandle,
                                    (pHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK),
                                    pHandle->interruptOutPipeDataBuffer, pHandle->interruptOutPipeDataLen);
                                if (kStatus_USB_Success != error)
                                {
                                    usb_device_endpoint_callback_message_struct_t endpointCallbackMessage;
                                    endpointCallbackMessage.buffer  = pHandle->interruptOutPipeDataBuffer;
                                    endpointCallbackMessage.length  = pHandle->interruptOutPipeDataLen;
                                    endpointCallbackMessage.isSetup = 0U;
                                    (void)USB_DeviceWeigherInterruptOut(pHandle->deviceHandle, (void *)&endpointCallbackMessage,
                                                                    handle);
                                }
                                pHandle->interruptOutPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
                                pHandle->interruptOutPipeDataLen    = 0U;
                            }
                        }
                    }
                }
            }
            break;
            
        case kUSB_DeviceClassEventClassRequest:
        {
            /* handle the hobart printer class specific request. */
            usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;

            if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) !=
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                break;
            }

            if ((controlRequest->setup->wIndex & 0xFFU) != pHandle->interfaceNumber)
            {
                break;
            }

            error = kStatus_USB_InvalidRequest;
            switch (controlRequest->setup->bRequest)
            {
                default:
                    /* no action, return kStatus_USB_InvalidRequest */
                    break;
            }
        }
        break;
        default:
            /*no action*/
            break;
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceHBWeigherInit( uint8_t controllerId, 
                                                usb_device_class_config_struct_t *config, 
                                                class_handle_t *handle )

      \brief
        This function is used to initialize the hobart weigher class.

      \param controllerId  The controller id of the USB IP.
                             returned from usb_device_class_config_struct_t::classHandle.
      \param config   The class configuration information.                        
      \param handle   used to return pointer of the hobart weigher class handle 
                        to the caller. 
                 
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceHBWeigherInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle)
{
    usb_device_weigher_t *pHandle;
    usb_status_t error;

    /* allocate a hobart weigher class handle. */
    error = USB_DeviceWeigherAllocateHandle(&pHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    /* Get the device handle according to the controller id. */
    error = USB_DeviceClassGetDeviceHandle(controllerId, &pHandle->deviceHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    if (NULL == pHandle->deviceHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* Save the configuration of the class. */
    pHandle->configStruct = config;
    /* Clear the configuration value. */
    pHandle->configuration = 0U;
    pHandle->alternate     = 0xffU;

    *handle = (class_handle_t)pHandle;
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceHBWeigherDeinit( class_handle_t handle )

      \brief
        This function is used to de-initialize the hobart weigher class.
                       
      \param handle   hobart weigher class handle 

                 
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceHBWeigherDeinit(class_handle_t handle)
{
    usb_device_weigher_t *pHandle;
    usb_status_t error;

    pHandle = (usb_device_weigher_t *)handle;

    if (NULL == pHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* De-initialzie the endpoints. */
    error = USB_DeviceWeigherEndpointsDeinit(pHandle);
    /* Free the hid class handle. */
    (void)USB_DeviceWeigherFreeHandle(pHandle);
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceHBWeigherSend( class_handle_t handle, 
                                                uint8_t ep, uint8_t *buffer, 
                                                uint32_t length )

      \brief
        This function is used to send data through a specified endpoint.
                       
      \param handle     The hobart weigher class handle.
      \param ep         Endpoint index.
      \param buffer     The memory address to hold the data need to be sent.
      \param length     The data length need to be sent.
                 
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceHBWeigherSend( class_handle_t handle, uint8_t ep, 
                                      uint8_t *buffer, uint32_t length )
{
    usb_device_weigher_t *pHandle;
    usb_status_t error = kStatus_USB_Error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    pHandle = (usb_device_weigher_t *)handle;

    if (0U != pHandle->interruptInPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    pHandle->interruptInPipeBusy = 1U;

    if (0U != pHandle->interruptInPipeStall)
    {
        pHandle->interruptInPipeDataBuffer = buffer;
        pHandle->interruptInPipeDataLen    = length;
        return kStatus_USB_Success;
    }
    error = USB_DeviceSendRequest(pHandle->deviceHandle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        pHandle->interruptInPipeBusy = 0U;
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceHBWeigherRecv( class_handle_t handle, uint8_t ep, 
                                                uint8_t *buffer, uint32_t length )

      \brief
        This function is used to receive data through a specified endpoint.
                       
      \param handle     The hobart weigher class handle.
      \param ep         Endpoint index.
      \param buffer     The memory address to save the received data.
      \param length     The data length want to be received.
                 
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceHBWeigherRecv( class_handle_t handle, uint8_t ep, 
                                      uint8_t *buffer, uint32_t length )
{
    usb_device_weigher_t *pHandle;
    usb_status_t error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    pHandle = (usb_device_weigher_t *)handle;

    if (0U != pHandle->interruptOutPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    pHandle->interruptOutPipeBusy = 1U;

    if (0U != pHandle->interruptOutPipeStall)
    {
        pHandle->interruptOutPipeDataBuffer = buffer;
        pHandle->interruptOutPipeDataLen    = length;
        return kStatus_USB_Success;
    }
    error = USB_DeviceRecvRequest(pHandle->deviceHandle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        pHandle->interruptOutPipeBusy = 0U;
    }
    return error;
}
#endif