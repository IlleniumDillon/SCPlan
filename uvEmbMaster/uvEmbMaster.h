#ifndef UVEMBMASTER_H
#define UVEMBMASTER_H

#include <iostream>

#include "libusb.h"

class UvEmbMaster
{
public:
    UvEmbMaster();
    ~UvEmbMaster();

    void reinit();

    size_t read_regCh(void* pdata, size_t size);
    size_t write_regCh(void* pdata, size_t size);
    size_t read_burCh(void* pdata, size_t size);
    size_t write_burCh(void* pdata, size_t size);
private:
    size_t read(void* pdata, size_t size, uint8_t ep);
    size_t write(void* pdata, size_t size, uint8_t ep);

    bool initDone = false;
    struct libusb_device_handle *devh = nullptr;
};

#endif // UVEMBMASTER_H