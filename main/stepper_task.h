#pragma once

#include "stepper_driver_tmc2208.h"

typedef struct stepper_conf_s {
    const char* name;
    uint32_t speed;  
    stepper_driver_tmc2208_conf_t stepper_driver_conf;
} stepper_conf_t;

#ifdef __cplusplus
extern "C" {
#endif
     void stepper_task(void *pvParameter);
#ifdef __cplusplus
}
#endif
