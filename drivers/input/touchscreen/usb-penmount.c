/*******************************************************
*
*  PenMount USB TouchScreen Driver
*
*  Copyright (c) 2011 PenMount Touch Solutions <penmount@seed.net.tw>
*
*******************************************************/

/*******************************************************
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation; either version 2 of the License, or (at your option)
* any later version.
*******************************************************/

////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#else
#include <linux/input.h>
#endif
////////////////////////////////////////////////////////
#define PMUSB_DRIVER_DESC "PenMount USB TouchScreen Driver"
////////////////////////////////////////////////////////
#define PENMOUNT_MAXTOUCH   16
#define PENMOUNT_MTPROTO_A  0
#define PENMOUNT_MTPROTO_B  1
#define PENMOUNT_MAXTRACKID 0xFFFF
#define PENMOUNT_BTN        BTN_TOUCH

#ifndef ABS_MT_SLOT
#define PENMOUNT_MTPROTO   PENMOUNT_MTPROTO_A
#else
#define PENMOUNT_MTPROTO   PENMOUNT_MTPROTO_B
#endif

#ifndef USB_VENDOR_ID_PENMOUNT
#define USB_VENDOR_ID_PENMOUNT          0x14E1
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_M1
#define USB_DEVICE_ID_PENMOUNT_M1       0x6250
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_P1
#define USB_DEVICE_ID_PENMOUNT_P1       0x3000
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_P2
#define USB_DEVICE_ID_PENMOUNT_P2       0x3500
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_6000
#define USB_DEVICE_ID_PENMOUNT_6000     0x6000
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_5000
#define USB_DEVICE_ID_PENMOUNT_5000     0x5000
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_9000
#define USB_DEVICE_ID_PENMOUNT_9000     0x9000
#endif

//------------------------------------------------------
struct strPMTOUCH
{
    __u8  bUpdated    ;
    __s32 TrackID     ;
    __u8  Slot        ;
    __u8  bTouch      ;
    __u8  bTouching   ;
    __u16 X           ;
    __u16 Y           ;
    __u16 LastX       ;
    __u16 LastY       ;
    __u8  LastState   ;

} ;
//------------------------------------------------------
struct strPMUSB
{
    struct urb        *pUrb             ;
    struct usb_device *pUsbDevice       ;
    dma_addr_t         DmaAddress       ;
} ;
//------------------------------------------------------
struct strPENMOUNT
{
    __u8               MaxTouch                 ;
    __u8               MTProtocol               ;
    __u16              Model                    ;
    char               szDeviceName[128]        ;
    char               szPhysDevice[64]         ;
    struct input_dev  *pInputDev                ;
    __u16              ResolutionX              ;
    __u16              ResolutionY              ;
    __u8              *pInputBuffer             ;
    __u8               cbInputReport            ;
    __s32              TrackIDCount             ;
    struct strPMTOUCH *pMainTouch               ;
    struct strPMTOUCH  Touch[PENMOUNT_MAXTOUCH] ;
    struct strPMUSB    USB                      ;
} ;
//------------------------------------------------------
static
int pmInputDev_Open ( struct input_dev *pInputDev )
{
    struct strPENMOUNT *pPenMount = ( struct strPENMOUNT * ) input_get_drvdata ( pInputDev ) ;

    if ( pPenMount == NULL )
        return -ENXIO ;

    pPenMount->USB.pUrb->dev = pPenMount->USB.pUsbDevice ;

    if ( usb_submit_urb ( pPenMount->USB.pUrb, GFP_KERNEL) )
        return -EIO ;

    return 0 ;
}
//------------------------------------------------------
static
void pmInputDev_Close ( struct input_dev *pInputDev )
{
    struct strPENMOUNT *pPenMount = ( struct strPENMOUNT * ) input_get_drvdata ( pInputDev ) ;

    if ( pPenMount == NULL )
        return ;

    usb_kill_urb ( pPenMount->USB.pUrb ) ;

    return ;
}
//------------------------------------------------------
static
void PenMount_InitInputDevice ( struct strPENMOUNT *pPenMount ,
struct input_dev   *pInputDev )
{
    int i = 0 ;
    for ( i = 0 ; i < PENMOUNT_MAXTOUCH ; i++ )
    {
        pPenMount->Touch[i].Slot = i ;
    }
    pInputDev->evbit[0]   = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    pInputDev->keybit[BIT_WORD(PENMOUNT_BTN)] = BIT_MASK(PENMOUNT_BTN) ;

    switch ( pPenMount->Model )
    {
    case  USB_DEVICE_ID_PENMOUNT_M1 :
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
        pPenMount->MaxTouch = 16 ;
        pPenMount->MTProtocol = PENMOUNT_MTPROTO ;
        switch ( pPenMount->MTProtocol )
        {
        case PENMOUNT_MTPROTO_A:
            input_set_abs_params  ( pInputDev, ABS_MT_TOUCH_MAJOR, 0, 5    , 0, 0 ) ;
            break ;
        case PENMOUNT_MTPROTO_B:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
            input_mt_init_slots   ( pInputDev, pPenMount->MaxTouch ) ;
#elif defined(ABS_MT_SLOT)
            input_mt_create_slots ( pInputDev, pPenMount->MaxTouch ) ;
            input_set_abs_params  ( pInputDev, ABS_MT_TRACKING_ID, 0, PENMOUNT_MAXTRACKID , 0, 0 ) ;
#endif
            break ;
        }
        input_set_abs_params  ( pInputDev, ABS_MT_POSITION_X , 0, 0x3FF, 0, 0 ) ;
        input_set_abs_params  ( pInputDev, ABS_MT_POSITION_Y , 0, 0x3FF, 0, 0 ) ;
#else
        pPenMount->MaxTouch = 1 ;
#endif
        input_set_abs_params ( pInputDev, ABS_X, 0, 0x3FF, 0, 0 ) ;
        input_set_abs_params ( pInputDev, ABS_Y, 0, 0x3FF, 0, 0 ) ;		
        break ;
    case  USB_DEVICE_ID_PENMOUNT_P1 :
    case  USB_DEVICE_ID_PENMOUNT_P2 :
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
        pPenMount->MaxTouch = 2 ;
        pPenMount->MTProtocol = PENMOUNT_MTPROTO ;
        switch ( pPenMount->MTProtocol )
        {
        case PENMOUNT_MTPROTO_A:
            input_set_abs_params  ( pInputDev, ABS_MT_TOUCH_MAJOR, 0, 5    , 0, 0 ) ;
            break ;
        case PENMOUNT_MTPROTO_B:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
            input_mt_init_slots   ( pInputDev, pPenMount->MaxTouch ) ;
#elif defined(ABS_MT_SLOT)
            input_mt_create_slots ( pInputDev, pPenMount->MaxTouch ) ;
            input_set_abs_params  ( pInputDev, ABS_MT_TRACKING_ID, 0, PENMOUNT_MAXTRACKID , 0, 0 ) ;
#endif
            break ;
        }
        input_set_abs_params  ( pInputDev, ABS_MT_POSITION_X , 0, 0x7FF, 0, 0 ) ;
        input_set_abs_params  ( pInputDev, ABS_MT_POSITION_Y , 0, 0x7FF, 0, 0 ) ;
#else
        pPenMount->MaxTouch = 1 ;
#endif
        input_set_abs_params ( pInputDev, ABS_X, 0, 0x7FF, 0, 0 ) ;
        input_set_abs_params ( pInputDev, ABS_Y, 0, 0x7FF, 0, 0 ) ;
        break ;
    case USB_DEVICE_ID_PENMOUNT_5000 :
    case USB_DEVICE_ID_PENMOUNT_6000 :
    case USB_DEVICE_ID_PENMOUNT_9000 :
        pPenMount->MaxTouch = 1 ;
        input_set_abs_params ( pInputDev, ABS_X, 0, 0x3FF, 0, 0 ) ;
        input_set_abs_params ( pInputDev, ABS_Y, 0, 0x3FF, 0, 0 ) ;
        break ;
    }
    return ;
}
//------------------------------------------------------
static
void PenMount_ProcessEvent ( struct input_dev   *pInputDev ,
struct strPENMOUNT *pPenMount ,
struct strPMTOUCH  *pTouch    )
{
    if ( pTouch->bTouch )
    {
        if ( !pTouch->bTouching )
        {
            if ( ( pPenMount->MaxTouch == 1 ) && ( pTouch->Slot == 0 ) )
                input_report_key ( pInputDev, PENMOUNT_BTN , 1 ) ;
            pTouch->bTouching = 1 ;
        }
    }
    else
    {
        if ( pTouch->bTouching )
        {
            if ( ( pPenMount->MaxTouch == 1 ) && ( pTouch->Slot == 0 ) )
                input_report_key ( pInputDev, PENMOUNT_BTN , 0 ) ;
            pTouch->bTouching = 0 ;
        }
    }

    if ( ( pPenMount->MaxTouch == 1 ) && ( pTouch->Slot == 0 ) )
    {
        input_report_abs ( pInputDev, ABS_X , pTouch->X ) ;
        input_report_abs ( pInputDev, ABS_Y , pTouch->Y ) ;
        input_sync       ( pInputDev ) ;
    }

    pTouch->bTouch = 0 ;

    return ;
}
//------------------------------------------------------
static
void PenMount_ProcessMTEvent ( struct input_dev   *pInputDev ,
struct strPENMOUNT *pPenMount )
{
    __u8 i = 0 ;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
    if ( pPenMount->MaxTouch > 1 )
    {
        __u8         TouchCount = 0 ;

        switch ( pPenMount->MTProtocol )
        {
        case PENMOUNT_MTPROTO_A :
            for ( i = 0 ; i < pPenMount->MaxTouch ; i++ )
            {
                if ( ( pPenMount->pMainTouch == NULL ) && ( pPenMount->Touch[i].bTouching ) )
                {
                    pPenMount->pMainTouch = &pPenMount->Touch[i] ;
                }
                if ( pPenMount->Touch[i].bTouching )
                {
                    input_report_abs ( pInputDev, ABS_MT_TOUCH_MAJOR , 2 ) ;
                    input_report_abs ( pInputDev, ABS_MT_POSITION_X  , pPenMount->Touch[i].X ) ;
                    input_report_abs ( pInputDev, ABS_MT_POSITION_Y  , pPenMount->Touch[i].Y ) ;
                    input_mt_sync    ( pInputDev ) ;
                    TouchCount++ ;
                }
            }

            if ( !TouchCount )
                input_mt_sync ( pInputDev ) ;

            break ;
        case PENMOUNT_MTPROTO_B :
#ifdef ABS_MT_SLOT
            for ( i = 0 ; i < pPenMount->MaxTouch ; i++ )
            {
                if ( ( pPenMount->pMainTouch == NULL ) && ( pPenMount->Touch[i].bTouching ) )
                    pPenMount->pMainTouch = &pPenMount->Touch[i] ;

                if ( ( pPenMount->Touch[i].X != pPenMount->Touch[i].LastX )
                    || ( pPenMount->Touch[i].Y != pPenMount->Touch[i].LastY )
                    || ( pPenMount->Touch[i].bTouching != pPenMount->Touch[i].LastState ) )
                {
                    input_mt_slot ( pInputDev, i ) ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
                    input_mt_report_slot_state ( pInputDev, MT_TOOL_FINGER, pPenMount->Touch[i].bTouching ) ;
#endif
                    if ( pPenMount->Touch[i].bTouching )
                    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
                        if ( pPenMount->Touch[i].TrackID < 0 )
                            pPenMount->Touch[i].TrackID = ( pPenMount->TrackIDCount++ & 0xFFFF ) ;
                        input_report_abs ( pInputDev, ABS_MT_TRACKING_ID , pPenMount->Touch[i].TrackID ) ;
#endif
                        if ( pPenMount->Touch[i].X != pPenMount->Touch[i].LastX )
                        {
                            input_report_abs ( pInputDev, ABS_MT_POSITION_X  , pPenMount->Touch[i].X ) ;
                            pPenMount->Touch[i].LastX = pPenMount->Touch[i].X ;
                        }
                        if ( pPenMount->Touch[i].Y != pPenMount->Touch[i].LastY )
                        {
                            input_report_abs ( pInputDev, ABS_MT_POSITION_Y  , pPenMount->Touch[i].Y ) ;
                            pPenMount->Touch[i].LastY = pPenMount->Touch[i].Y ;
                        }
                        if ( pPenMount->Touch[i].bTouching != pPenMount->Touch[i].LastState )
                        {
                            pPenMount->Touch[i].LastState = pPenMount->Touch[i].bTouching ;
                        }
                    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
                    else if ( pPenMount->Touch[i].TrackID != -1 )
                    {
                        pPenMount->Touch[i].TrackID = -1 ;
                        input_report_abs ( pInputDev, ABS_MT_TRACKING_ID , pPenMount->Touch[i].TrackID ) ;
                    }
#endif
                }
            }
#endif
            break ;
        }

        // Single-Touch Emulation
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
        input_mt_report_pointer_emulation ( pInputDev, true ) ;
#else
        if ( pPenMount->pMainTouch )
        {
            if ( pPenMount->pMainTouch->bTouching )
            {
                input_report_key ( pInputDev, PENMOUNT_BTN , 1 ) ;
                input_report_abs ( pInputDev, ABS_X  , pPenMount->pMainTouch->X ) ;
                input_report_abs ( pInputDev, ABS_Y  , pPenMount->pMainTouch->Y ) ;
            }
            else
            {
                input_report_key ( pInputDev, PENMOUNT_BTN , 0 ) ; 
                pPenMount->pMainTouch = NULL ;
            }
        }
#endif
        input_sync ( pInputDev ) ;
    }
#endif

    for ( i = 0 ; i < pPenMount->MaxTouch ; i++ )
        pPenMount->Touch[i].bUpdated = 0 ;

    return ;
}         
//------------------------------------------------------
static
void UsbPenMount_ReadComplete ( struct urb *pUrb )
{
    struct strPENMOUNT *pPenMount = pUrb->context ;
    struct input_dev   *pInputDev = pPenMount->pInputDev ;
    __u8                Slot = 0 ; 
    __u8                i    = 0 ;
    __u8                bProcessEvents = 1 ;

    switch ( pUrb->status )
    {
    case 0 :
        switch ( pPenMount->Model )
        {
        default :
        case USB_DEVICE_ID_PENMOUNT_5000 :
        case USB_DEVICE_ID_PENMOUNT_6000 :
            pPenMount->Touch[Slot].bTouch = ( ( pPenMount->pInputBuffer[0] & 0xF0 ) == 0x70 ) ;
            pPenMount->Touch[Slot].X      = pPenMount->pInputBuffer[1] + ( pPenMount->pInputBuffer[2] << 8 ) ;
            pPenMount->Touch[Slot].Y      = pPenMount->pInputBuffer[3] + ( pPenMount->pInputBuffer[4] << 8 ) ;
            PenMount_ProcessEvent ( pInputDev , pPenMount , &pPenMount->Touch[Slot] ) ;
            break ;
        case USB_DEVICE_ID_PENMOUNT_M1  :
        case USB_DEVICE_ID_PENMOUNT_P1  :
        case USB_DEVICE_ID_PENMOUNT_P2  :
            Slot = ( pPenMount->pInputBuffer[0] & 0x0F ) ;
            pPenMount->Touch[Slot].bTouch = ( ( pPenMount->pInputBuffer[0] & 0xF0 ) == 0x70 ) ;
            pPenMount->Touch[Slot].X      = pPenMount->pInputBuffer[1] + ( pPenMount->pInputBuffer[2] << 8 ) ;
            pPenMount->Touch[Slot].Y      = pPenMount->pInputBuffer[3] + ( pPenMount->pInputBuffer[4] << 8 ) ;
            if ( pPenMount->Touch[Slot].bUpdated )
                PenMount_ProcessMTEvent ( pInputDev, pPenMount ) ;
            pPenMount->Touch[Slot].bUpdated = 1 ;
            PenMount_ProcessEvent ( pInputDev , pPenMount , &pPenMount->Touch[Slot] ) ;
            for ( i = 0 ; i < pPenMount->MaxTouch ; i++ )
                if ( ( !pPenMount->Touch[i].bUpdated ) && ( pPenMount->Touch[i].bTouching ) )
                    bProcessEvents = 0 ;
            if ( bProcessEvents )
                PenMount_ProcessMTEvent ( pInputDev, pPenMount ) ;
            break ;
        }
    default :
        usb_submit_urb ( pUrb, GFP_ATOMIC ) ;
        break ;
    case -ETIME      : /* URB Time Out */
    case -ECONNRESET :
    case -ENOENT     :
    case -ESHUTDOWN  :
        break ;

    }
    return ;
}
//------------------------------------------------------
static
int UsbPenMount_Probe (       struct usb_interface *pUsbInterface ,
                       const struct usb_device_id *pUsbDeviceID  )
{
    struct usb_device              *pUsbDevice     = NULL ;
    struct usb_endpoint_descriptor *pUsbEndPoint   = NULL ;
    struct input_dev               *pInputDev      = NULL ;
    struct strPENMOUNT             *pPenMount      = NULL ;

    pPenMount = kzalloc ( sizeof(struct strPENMOUNT), GFP_KERNEL ) ;
    if ( pPenMount == NULL )
        return -ENOMEM ;

    memset ( pPenMount , 0 , sizeof(struct strPENMOUNT) ) ;	

    // Initialize the USB Device
    pUsbDevice   =  interface_to_usbdev ( pUsbInterface ) ;	
    pUsbEndPoint = &pUsbInterface->cur_altsetting->endpoint[0].desc ;

    pPenMount->USB.pUsbDevice = pUsbDevice   ;

    usb_make_path ( pUsbDevice, pPenMount->szPhysDevice, sizeof(pPenMount->szPhysDevice) ) ;
    strlcat ( pPenMount->szPhysDevice, "/input0", sizeof(pPenMount->szPhysDevice) ) ;

    pPenMount->USB.pUrb       = usb_alloc_urb ( 0, GFP_KERNEL ) ;
    if ( pPenMount->USB.pUrb == NULL )
    {
        kfree ( pPenMount ) ;
        return -ENOMEM ;
    }
    pPenMount->cbInputReport = 5 ;

    pPenMount->pInputBuffer = (unsigned char *)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        usb_alloc_coherent
#else
        usb_buffer_alloc
#endif
        ( pUsbDevice                   ,
          pPenMount->cbInputReport     ,
          GFP_KERNEL                   ,
          &pPenMount->USB.DmaAddress ) ;
    if ( pPenMount->pInputBuffer == NULL )
    {
        kfree ( pPenMount ) ;
        return -ENOMEM ;
    }

    usb_fill_int_urb ( pPenMount->USB.pUrb          ,
        pUsbDevice                   ,
        usb_rcvintpipe ( pUsbDevice, pUsbEndPoint->bEndpointAddress ) ,
        pPenMount->pInputBuffer      ,
        pPenMount->cbInputReport     ,
        UsbPenMount_ReadComplete     ,
        pPenMount                    ,
        pUsbEndPoint->bInterval    ) ;

    pPenMount->USB.pUrb->dev             = pUsbDevice                ;
    pPenMount->USB.pUrb->transfer_dma    = pPenMount->USB.DmaAddress ;
    pPenMount->USB.pUrb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP   ;

    usb_set_intfdata  ( pUsbInterface, pPenMount ) ;

    // Initialize the Input Device
    pInputDev = input_allocate_device() ;
    if ( pInputDev == NULL )
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        usb_free_coherent
#else
        usb_buffer_free
#endif
            ( pUsbDevice                  ,
              pPenMount->cbInputReport    ,
              pPenMount->pInputBuffer     ,
              pPenMount->USB.DmaAddress ) ;

        usb_free_urb ( pPenMount->USB.pUrb ) ;
        kfree ( pPenMount ) ;
        return -ENOMEM ;
    }

    usb_to_input_id ( pUsbDevice, &pInputDev->id ) ;
    pPenMount->Model         = pInputDev->id.product ;
    switch ( pPenMount->Model )
    {
    case  USB_DEVICE_ID_PENMOUNT_M1 :
        snprintf ( pPenMount->szDeviceName, sizeof(pPenMount->szDeviceName), "PenMount MF TouchScreen" ) ;
        break ;
    case  USB_DEVICE_ID_PENMOUNT_P1 :
    case  USB_DEVICE_ID_PENMOUNT_P2 :
        snprintf ( pPenMount->szDeviceName, sizeof(pPenMount->szDeviceName), "PenMount PCI TouchScreen" ) ;
        break ;
    case USB_DEVICE_ID_PENMOUNT_5000 :
    case USB_DEVICE_ID_PENMOUNT_6000 :
        snprintf ( pPenMount->szDeviceName, sizeof(pPenMount->szDeviceName), "PenMount USB %X TouchScreen", pPenMount->Model ) ;
        break ;
    }
    PenMount_InitInputDevice ( pPenMount , pInputDev ) ;

    pInputDev->name       =  pPenMount->szDeviceName ;
    pInputDev->phys       =  pPenMount->szPhysDevice ;
    pInputDev->dev.parent = &pUsbInterface->dev      ;
    pInputDev->open       =  pmInputDev_Open         ;
    pInputDev->close      =  pmInputDev_Close        ;

    input_set_drvdata ( pInputDev, pPenMount ) ;
    input_register_device ( pInputDev ) ;

    pPenMount->pInputDev  = pInputDev ;

    return 0 ;
}
//------------------------------------------------------
static 
void UsbPenMount_Disconnect ( struct usb_interface *pUsbInterface )
{
    struct strPENMOUNT *pPenMount = ( struct strPENMOUNT * ) usb_get_intfdata ( pUsbInterface ) ;

    if ( pPenMount == NULL )
        return ;

    usb_kill_urb ( pPenMount->USB.pUrb ) ;
    usb_free_urb ( pPenMount->USB.pUrb ) ;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
    usb_free_coherent
#else
    usb_buffer_free
#endif
        ( interface_to_usbdev(pUsbInterface) ,
          pPenMount->cbInputReport           ,
          pPenMount->pInputBuffer            ,
          pPenMount->USB.DmaAddress        ) ;

    kfree ( pPenMount->pInputBuffer ) ;

    input_unregister_device ( pPenMount->pInputDev ) ;

    usb_set_intfdata ( pUsbInterface, NULL ) ;

    kfree ( pPenMount ) ;

    return ;
}
//------------------------------------------------------
static
struct usb_device_id USBPENMOUNT_DEVICES[] =
{
    { USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_5000) },
    { USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_6000) },
    { USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_P1)   },
    { USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_P2)   },
    { USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_M1)   },
    {}
} ;
//------------------------------------------------------
static struct usb_driver USBPENMOUNT_DRIVER =
{
    .name       = "USB-PenMount"         ,
    .probe      = UsbPenMount_Probe      ,
    .disconnect = UsbPenMount_Disconnect ,
    .id_table   = USBPENMOUNT_DEVICES    ,
};
////////////////////////////////////////////////////////
static
int __init UsbPenMount_Init(void)
{
    return usb_register ( &USBPENMOUNT_DRIVER ) ;
}
//------------------------------------------------------
static
void __exit UsbPenMount_Exit ( void )
{
    usb_deregister ( &USBPENMOUNT_DRIVER ) ;
    return ;
}
module_init(UsbPenMount_Init);
module_exit(UsbPenMount_Exit);
////////////////////////////////////////////////////////
MODULE_AUTHOR("PenMount Touch Solutions <penmount@seed.net.tw>");
MODULE_DESCRIPTION(PMUSB_DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(usb, USBPENMOUNT_DEVICES);
