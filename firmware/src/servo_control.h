#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);
void servo_home(void);

void servo_write_all(const int angles_deg[NUM_SERVOS]);
void servo_write_single(int index, int logical_deg);

void servo_test_serial(void);

#ifdef __cplusplus
}
#endif

#endif
