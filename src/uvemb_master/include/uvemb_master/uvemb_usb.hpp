#ifndef UVEMB_USB_HPP
#define UVEMB_USB_HPP

#include <iostream>

class UvEmbUsb_private;

class UvEmbUsb
{
public:
    UvEmbUsb();
    ~UvEmbUsb();

    bool init();

    size_t read_regCh(void* pdata, size_t size);
    size_t write_regCh(void* pdata, size_t size);
    size_t read_burCh(void* pdata, size_t size);
    size_t write_burCh(void* pdata, size_t size);
private:
    UvEmbUsb_private* impl;
};

#endif // UVEMB_USB_HPP