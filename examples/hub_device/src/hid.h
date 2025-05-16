#pragma once

#include <stdint.h>
#include "usbd.h"

#ifdef __cplusplus
extern "C" {
#endif

usbd_handle_t* hid_init(usbd_hw_type_t hw_type);
void hid_task(usbd_handle_t* handle);

#ifdef __cplusplus
}
#endif