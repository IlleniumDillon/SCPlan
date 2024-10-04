#include "uvEmbMaster.h"

#define USB_VID   0xCafe
#define USB_BCD   0x0200
#define USB_PID   0x4002

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#define EPNUM_CDC_1_NOTIF   0x83
#define EPNUM_CDC_1_OUT     0x04
#define EPNUM_CDC_1_IN      0x84

UvEmbMaster::UvEmbMaster()
{
    reinit();
}

UvEmbMaster::~UvEmbMaster()
{
    if (devh != nullptr)
    {
        libusb_release_interface(devh, 0);
        libusb_close(devh);
        devh = nullptr;
    }
    libusb_exit(NULL);
}

void UvEmbMaster::reinit()
{
    initDone = false;
    if (devh != nullptr)
    {
        libusb_release_interface(devh, 0);
        libusb_close(devh);
        devh = nullptr;
    }

    int rc;
    rc = libusb_init(NULL);
    if (rc < 0) 
    {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(rc) << std::endl;
        return;
    }
    devh = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID);
    if (devh == NULL) 
    {
        std::cerr << "Failed to open device" << std::endl;
        return;
    }

    for (int if_num = 0; if_num < 4; if_num++) 
    {
        if (libusb_kernel_driver_active(devh, if_num)) 
        {
            rc = libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) 
        {
            std::cerr << "Failed to claim interface " << if_num << ": " << libusb_error_name(rc) << std::endl;
            return;
        }
    }

    initDone = true;
}

size_t UvEmbMaster::read_regCh(void *pdata, size_t size)
{
    return read(pdata, size, EPNUM_CDC_0_IN);
}

size_t UvEmbMaster::write_regCh(void *pdata, size_t size)
{
    return write(pdata, size, EPNUM_CDC_0_OUT);
}

size_t UvEmbMaster::read_burCh(void *pdata, size_t size)
{
    return read(pdata, size, EPNUM_CDC_1_IN);
}

size_t UvEmbMaster::write_burCh(void *pdata, size_t size)
{
    return write(pdata, size, EPNUM_CDC_1_OUT);
}

size_t UvEmbMaster::read(void* pdata, size_t size, uint8_t ep)
{
    if (initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(devh, ep, (uint8_t*)pdata, size, &actual_length, 0);
        return actual_length;
    }
    else
    {
        return 0;
    }
}

size_t UvEmbMaster::write(void* pdata, size_t size, uint8_t ep)
{
    if (initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(devh, ep, (uint8_t*)pdata, size, &actual_length, 0);
        return actual_length;
    }
    else
    {
        return 0;
    }
}
