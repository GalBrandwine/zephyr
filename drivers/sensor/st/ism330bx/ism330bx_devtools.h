#ifndef ISM330BX_DEVTOOLS_H
#define ISM330BX_DEVTOOLS_H
#include <zephyr/logging/log.h>
#include "ism330bx.h"

// Helper functions
void ism330bx_helper_print_accel_odr(ism330bx_xl_data_rate_t *data_rate);
void ism330bx_helper_print_accel_mode(ism330bx_xl_mode_t *mode);
void ism330bx_helper_print_accel_full_scale(ism330bx_xl_full_scale_t *full_scale);
void ism330bx_helper_print_gyro_odr(ism330bx_gy_data_rate_t *data_rate);
void ism330bx_helper_print_gyro_mode(ism330bx_gy_mode_t *mode);
void ism330bx_helper_print_gyro_full_scale(ism330bx_gy_full_scale_t *full_scale);

void ism330bx_helper_print_all_sources(const ism330bx_all_sources_t *all_sources);
void ism330bx_helper_print_orientation_interrupt_flags(const ism330bx_all_sources_t *all_sources);
void ism330bx_helper_print_int1_route_config(ism330bx_pin_int_route_t *pin_int_route);
int  ism330bx_helper_print_accel_config(stmdev_ctx_t *ctx);
int  ism330bx_helper_print_gyro_config(stmdev_ctx_t *ctx);

#endif // ISM330BX_DEVTOOLS_H