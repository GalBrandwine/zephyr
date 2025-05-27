#include "ism330bx_devtools.h"

LOG_MODULE_DECLARE(ism330bx, CONFIG_ISM330BX_LOG_LEVEL);

void ism330bx_helper_print_all_sources(const ism330bx_all_sources_t *all_sources)
{
    LOG_DBG("all_sources:");
    LOG_DBG("all_sources.fifo_th: %d", all_sources->fifo_th);
    LOG_DBG("all_sources.drdy_xl: %d", all_sources->drdy_xl);
    LOG_DBG("all_sources.drdy_gy: %d", all_sources->drdy_gy);
    LOG_DBG("all_sources.drdy_temp: %d", all_sources->drdy_temp);
    LOG_DBG("all_sources.drdy_ah_qvar: %d", all_sources->drdy_ah_qvar);
    LOG_DBG("all_sources.gy_settling: %d", all_sources->gy_settling);
    LOG_DBG("all_sources.den_flag: %d", all_sources->den_flag);
    LOG_DBG("all_sources.timestamp: %d", all_sources->timestamp);
    LOG_DBG("all_sources.free_fall: %d", all_sources->free_fall);
    LOG_DBG("all_sources.wake_up: %d", all_sources->wake_up);
    LOG_DBG("all_sources.wake_up_z: %d", all_sources->wake_up_z);
    LOG_DBG("all_sources.wake_up_y: %d", all_sources->wake_up_y);
    LOG_DBG("all_sources.wake_up_x: %d", all_sources->wake_up_x);
    LOG_DBG("all_sources.single_tap: %d", all_sources->single_tap);
    LOG_DBG("all_sources.double_tap: %d", all_sources->double_tap);
    LOG_DBG("all_sources.tap_z: %d", all_sources->tap_z);
    LOG_DBG("all_sources.tap_y: %d", all_sources->tap_y);
    LOG_DBG("all_sources.tap_x: %d", all_sources->tap_x);
    LOG_DBG("all_sources.tap_sign: %d", all_sources->tap_sign);
    LOG_DBG("all_sources.six_d: %d", all_sources->six_d);
    LOG_DBG("all_sources.six_d_xl: %d", all_sources->six_d_xl);
    LOG_DBG("all_sources.six_d_xh: %d", all_sources->six_d_xh);
    LOG_DBG("all_sources.six_d_yl: %d", all_sources->six_d_yl);
    LOG_DBG("all_sources.six_d_yh: %d", all_sources->six_d_yh);
    LOG_DBG("all_sources.six_d_zl: %d", all_sources->six_d_zl);
    LOG_DBG("all_sources.six_d_zh: %d", all_sources->six_d_zh);
    LOG_DBG("all_sources.sleep_change: %d", all_sources->sleep_change);
    LOG_DBG("all_sources.sleep_state: %d", all_sources->sleep_state);
    LOG_DBG("all_sources.step_detector: %d", all_sources->step_detector);
    LOG_DBG("all_sources.step_count_inc: %d", all_sources->step_count_inc);
    LOG_DBG("all_sources.step_count_overflow: %d", all_sources->step_count_overflow);
    LOG_DBG("all_sources.step_on_delta_time: %d", all_sources->step_on_delta_time);
    LOG_DBG("all_sources.emb_func_stand_by: %d", all_sources->emb_func_stand_by);
    LOG_DBG("all_sources.emb_func_time_exceed: %d", all_sources->emb_func_time_exceed);
    LOG_DBG("all_sources.tilt: %d", all_sources->tilt);
    LOG_DBG("all_sources.sig_mot: %d", all_sources->sig_mot);
    LOG_DBG("all_sources.fsm_lc: %d", all_sources->fsm_lc);
    LOG_DBG("all_sources.fsm1: %d", all_sources->fsm1);
    LOG_DBG("all_sources.fsm2: %d", all_sources->fsm2);
    LOG_DBG("all_sources.fsm3: %d", all_sources->fsm3);
    LOG_DBG("all_sources.fsm4: %d", all_sources->fsm4);
    LOG_DBG("all_sources.fsm5: %d", all_sources->fsm5);
    LOG_DBG("all_sources.fsm6: %d", all_sources->fsm6);
    LOG_DBG("all_sources.fsm7: %d", all_sources->fsm7);
    LOG_DBG("all_sources.fsm8: %d", all_sources->fsm8);
    LOG_DBG("all_sources.mlc1: %d", all_sources->mlc1);
    LOG_DBG("all_sources.mlc2: %d", all_sources->mlc2);
    LOG_DBG("all_sources.mlc3: %d", all_sources->mlc3);
    LOG_DBG("all_sources.mlc4: %d", all_sources->mlc4);
    LOG_DBG("all_sources.fifo_bdr: %d", all_sources->fifo_bdr);
    LOG_DBG("all_sources.fifo_full: %d", all_sources->fifo_full);
    LOG_DBG("all_sources.fifo_ovr: %d", all_sources->fifo_ovr);
    LOG_DBG("all_sources.fifo_th: %d", all_sources->fifo_th);
}
void ism330bx_helper_print_orientation_interrupt_flags(const ism330bx_all_sources_t *all_sources)
{
    LOG_DBG("all_sources.six_d: %d", all_sources->six_d);
    LOG_DBG("all_sources.six_d_xl: %d", all_sources->six_d_xl);
    LOG_DBG("all_sources.six_d_xh: %d", all_sources->six_d_xh);
    LOG_DBG("all_sources.six_d_yl: %d", all_sources->six_d_yl);
    LOG_DBG("all_sources.six_d_yh: %d", all_sources->six_d_yh);
    LOG_DBG("all_sources.six_d_zl: %d", all_sources->six_d_zl);
    LOG_DBG("all_sources.six_d_zh: %d", all_sources->six_d_zh);
}
void ism330bx_helper_print_int1_route_config(ism330bx_pin_int_route_t *pin_int_route)
{
    LOG_DBG("Reading routes interrupt signals on INT 1 pin");
    LOG_DBG("pin_int_route.boot: %d", pin_int_route->boot);
    LOG_DBG("pin_int_route.drdy_xl: %d", pin_int_route->drdy_xl);
    LOG_DBG("pin_int_route.drdy_gy: %d", pin_int_route->drdy_gy);
    LOG_DBG("pin_int_route.drdy_temp: %d", pin_int_route->drdy_temp);
    LOG_DBG("pin_int_route.drdy_ah_qvar: %d", pin_int_route->drdy_ah_qvar);
    LOG_DBG("pin_int_route.fifo_th: %d", pin_int_route->fifo_th);
    LOG_DBG("pin_int_route.fifo_ovr: %d", pin_int_route->fifo_ovr);
    LOG_DBG("pin_int_route.fifo_full: %d", pin_int_route->fifo_full);
    LOG_DBG("pin_int_route.fifo_bdr: %d", pin_int_route->fifo_bdr);
    LOG_DBG("pin_int_route.den_flag: %d", pin_int_route->den_flag);
    LOG_DBG("pin_int_route.timestamp: %d", pin_int_route->timestamp); // impact on int2 signals
    LOG_DBG("pin_int_route.six_d: %d", pin_int_route->six_d);
    LOG_DBG("pin_int_route.double_tap: %d", pin_int_route->double_tap);
    LOG_DBG("pin_int_route.free_fall: %d", pin_int_route->free_fall);
    LOG_DBG("pin_int_route.wake_up: %d", pin_int_route->wake_up);
    LOG_DBG("pin_int_route.single_tap: %d", pin_int_route->single_tap);
    LOG_DBG("pin_int_route.sleep_change: %d", pin_int_route->sleep_change);
    LOG_DBG("pin_int_route.sleep_status: %d", pin_int_route->sleep_status);
    LOG_DBG("pin_int_route.step_detector: %d", pin_int_route->step_detector);
    LOG_DBG("pin_int_route.step_count_overflow: %d", pin_int_route->step_count_overflow);
    LOG_DBG("pin_int_route.tilt: %d", pin_int_route->tilt);
    LOG_DBG("pin_int_route.sig_mot: %d", pin_int_route->sig_mot);
    LOG_DBG("pin_int_route.emb_func_stand_by: %d", pin_int_route->emb_func_stand_by); // impact on int2 signals
    LOG_DBG("pin_int_route.fsm_lc: %d", pin_int_route->fsm_lc);
    LOG_DBG("pin_int_route.fsm1: %d", pin_int_route->fsm1);
    LOG_DBG("pin_int_route.fsm2: %d", pin_int_route->fsm2);
    LOG_DBG("pin_int_route.fsm3: %d", pin_int_route->fsm3);
    LOG_DBG("pin_int_route.fsm4: %d", pin_int_route->fsm4);
    LOG_DBG("pin_int_route.fsm5: %d", pin_int_route->fsm5);
    LOG_DBG("pin_int_route.fsm6: %d", pin_int_route->fsm6);
    LOG_DBG("pin_int_route.fsm7: %d", pin_int_route->fsm7);
    LOG_DBG("pin_int_route.fsm8: %d", pin_int_route->fsm8);
    LOG_DBG("pin_int_route.mlc1: %d", pin_int_route->mlc1);
    LOG_DBG("pin_int_route.mlc2: %d", pin_int_route->mlc2);
    LOG_DBG("pin_int_route.mlc3: %d", pin_int_route->mlc3);
    LOG_DBG("pin_int_route.mlc4: %d", pin_int_route->mlc4);
}

void ism330bx_helper_print_gyro_mode(ism330bx_gy_mode_t *mode)
{
    switch (*mode)
    {
        case ISM330BX_GY_HIGH_PERFORMANCE_MD:
            LOG_DBG("gy mode: ISM330BX_GY_HIGH_PERFORMANCE_MD");
            break;
        case ISM330BX_GY_SLEEP_MD:
            LOG_DBG("gy mode: ISM330BX_GY_SLEEP_MD");
            break;
        case ISM330BX_GY_LOW_POWER_MD:
            LOG_DBG("gy mode: ISM330BX_GY_LOW_POWER_MD");
            break;
        default:
            LOG_ERR("Unknown gy mode");
    }
}
void ism330bx_helper_print_gyro_odr(ism330bx_gy_data_rate_t *data_rate)
{
    switch (*data_rate)
    {
        case ISM330BX_GY_ODR_OFF:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_OFF");
            break;
        case ISM330BX_GY_ODR_AT_7Hz5:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_7Hz5");
            break;
        case ISM330BX_GY_ODR_AT_15Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_15Hz");
            break;
        case ISM330BX_GY_ODR_AT_30Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_30Hz");
            break;
        case ISM330BX_GY_ODR_AT_60Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_60Hz");
            break;
        case ISM330BX_GY_ODR_AT_120Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_120Hz");
            break;
        case ISM330BX_GY_ODR_AT_240Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_240Hz");
            break;
        case ISM330BX_GY_ODR_AT_480Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_480Hz");
            break;
        case ISM330BX_GY_ODR_AT_960Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_960Hz");
            break;
        case ISM330BX_GY_ODR_AT_1920Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_1920Hz");
            break;
        case ISM330BX_GY_ODR_AT_3840Hz:
            LOG_DBG("gy data rate: ISM330BX_GY_ODR_AT_3840Hz");
            break;
        default:
            LOG_ERR("Unknown gy data rate");
    }
}
void ism330bx_helper_print_gyro_self_test(ism330bx_gy_self_test_t *self_test)
{
    switch (*self_test)
    {
        case ISM330BX_GY_ST_DISABLE:
            LOG_DBG("gy self test: ISM330BX_GY_ST_DISABLE");
            break;
        case ISM330BX_GY_ST_POSITIVE:
            LOG_DBG("gy self test: ISM330BX_GY_ST_POSITIVE");
            break;
        case ISM330BX_GY_ST_NEGATIVE:
            LOG_DBG("gy self test: ISM330BX_GY_ST_NEGATIVE");
            break;
        default:
            LOG_ERR("Unknown gy self test");
    }
}

void ism330bx_helper_print_gyro_full_scale(ism330bx_gy_full_scale_t *full_scale)
{

    switch (*full_scale)
    {
        case ISM330BX_125dps:
            LOG_DBG("gy full scale: ISM330BX_125dps");
            break;
        case ISM330BX_250dps:
            LOG_DBG("gy full scale: ISM330BX_250dps");
            break;
        case ISM330BX_500dps:
            LOG_DBG("gy full scale: ISM330BX_500dps");
            break;
        case ISM330BX_1000dps:
            LOG_DBG("gy full scale: ISM330BX_1000dps");
            break;
        case ISM330BX_2000dps:
            LOG_DBG("gy full scale: ISM330BX_2000dps");
            break;
        case ISM330BX_4000dps:
            LOG_DBG("gy full scale: ISM330BX_4000dps");
            break;
        default:
            LOG_ERR("Unknown gy full scale");
    }
}
void ism330bx_helper_print_accel_mode(ism330bx_xl_mode_t *mode)
{

    switch (*mode)
    {
        case ISM330BX_XL_HIGH_PERFORMANCE_MD:
            LOG_DBG("xl mode: ISM330BX_XL_HIGH_PERFORMANCE_MD");
            break;
        case ISM330BX_XL_HIGH_PERFORMANCE_TDM_MD:
            LOG_DBG("xl mode: ISM330BX_XL_HIGH_PERFORMANCE_TDM_MD");
            break;
        case ISM330BX_XL_LOW_POWER_2_AVG_MD:
            LOG_DBG("xl mode: ISM330BX_XL_LOW_POWER_2_AVG_MD");
            break;
        case ISM330BX_XL_LOW_POWER_4_AVG_MD:
            LOG_DBG("xl mode: ISM330BX_XL_LOW_POWER_4_AVG_MD");
            break;
        case ISM330BX_XL_LOW_POWER_8_AVG_MD:
            LOG_DBG("xl mode: ISM330BX_XL_LOW_POWER_8_AVG_MD");
            break;
        default:
            LOG_ERR("Unknown xl mode: %d", *mode);
    }
}
void ism330bx_helper_print_accel_odr(ism330bx_xl_data_rate_t *data_rate)
{

    switch (*data_rate)
    {
        case ISM330BX_XL_ODR_OFF:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_OFF");
            break;
        case ISM330BX_XL_ODR_AT_1Hz875:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_1Hz875");
            break;
        case ISM330BX_XL_ODR_AT_7Hz5:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_7Hz5");
            break;
        case ISM330BX_XL_ODR_AT_15Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_15Hz");
            break;
        case ISM330BX_XL_ODR_AT_30Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_30Hz");
            break;
        case ISM330BX_XL_ODR_AT_60Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_60Hz");
            break;
        case ISM330BX_XL_ODR_AT_120Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_120Hz");
            break;
        case ISM330BX_XL_ODR_AT_240Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_240Hz");
            break;
        case ISM330BX_XL_ODR_AT_480Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_480Hz");
            break;
        case ISM330BX_XL_ODR_AT_960Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_960Hz");
            break;
        case ISM330BX_XL_ODR_AT_1920Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_1920Hz");
            break;
        case ISM330BX_XL_ODR_AT_3840Hz:
            LOG_DBG("xl data rate: ISM330BX_XL_ODR_AT_3840Hz");
            break;
        default:
            LOG_ERR("Unknown xl data rate");
    }
}
void ism330bx_helper_print_accel_self_test(ism330bx_xl_self_test_t *self_test)
{

    switch (*self_test)
    {
        case ISM330BX_XL_ST_DISABLE:
            LOG_DBG("xl self test: ISM330BX_XL_ST_DISABLE");
            break;
        case ISM330BX_XL_ST_POSITIVE:
            LOG_DBG("xl self test: ISM330BX_XL_ST_POSITIVE");
            break;
        case ISM330BX_XL_ST_NEGATIVE:
            LOG_DBG("xl self test: ISM330BX_XL_ST_NEGATIVE");
            break;
        case ISM330BX_XL_ST_OFFSET_POS:
            LOG_DBG("xl self test: ISM330BX_XL_ST_OFFSET_POS");
            break;
        case ISM330BX_XL_ST_OFFSET_NEG:
            LOG_DBG("xl self test: ISM330BX_XL_ST_OFFSET_NEG");
            break;
        default:
            LOG_ERR("Unknown xl self test");
    }
}
void ism330bx_helper_print_accel_full_scale(ism330bx_xl_full_scale_t *full_scale)
{

    switch (*full_scale)
    {
        case ISM330BX_2g:
            LOG_DBG("xl full scale: ISM330BX_2g");
            break;
        case ISM330BX_4g:
            LOG_DBG("xl full scale: ISM330BX_4g");
            break;
        case ISM330BX_8g:
            LOG_DBG("xl full scale: ISM330BX_8g");
            break;
        default:
            LOG_ERR("Unknown xl full scale");
    }
}
int ism330bx_helper_print_accel_config(stmdev_ctx_t *ctx)
{
    // Get all accelerometer data
    // TODO: Later well decide if we need to get all data or just the one we need
    // TODO: We can store it in the data struct
    int ret = 0;
    LOG_DBG("Reading accelerometer config");

    ism330bx_xl_mode_t mode;
    ret = ism330bx_xl_mode_get(ctx, &mode);
    if (ret != 0)
    {
        LOG_ERR("Failed to get accelerometer mode");
        return -EIO;
    }
    ism330bx_helper_print_accel_mode(&mode);

    if (ret != 0)
    {
        LOG_ERR("Failed to get accelerometer mode");
        return -EIO;
    }

    ism330bx_xl_data_rate_t data_rate;
    ret = ism330bx_xl_data_rate_get(ctx, &data_rate);
    if (ret != 0)
    {
        LOG_ERR("Failed to get accelerometer data rate");
        return -EIO;
    }
    ism330bx_helper_print_accel_odr(&data_rate);

    ism330bx_xl_self_test_t self_test;
    ret = ism330bx_xl_self_test_get(ctx, &self_test);
    if (ret != 0)
    {
        LOG_ERR("Failed to get accelerometer self test");
        return -EIO;
    }
    ism330bx_helper_print_accel_self_test(&self_test);

    ism330bx_xl_full_scale_t full_scale;
    ret = ism330bx_xl_full_scale_get(ctx, &full_scale);
    if (ret != 0)
    {
        LOG_ERR("Failed to get accelerometer full scale");
        return -EIO;
    }
    ism330bx_helper_print_accel_full_scale(&full_scale);

    LOG_DBG("Finished reading accelerometer config");
    return EXIT_SUCCESS;
}

int ism330bx_helper_print_gyro_config(stmdev_ctx_t *ctx)
{
    // Get all gyroscope data
    LOG_DBG("Reading gyroscope config");
    ism330bx_gy_mode_t mode;
    int                ret = ism330bx_gy_mode_get(ctx, &mode);
    if (ret != 0)
    {
        LOG_ERR("Failed to get gyroscope mode");
        return -EIO;
    }
    ism330bx_helper_print_gyro_mode(&mode);

    ism330bx_gy_data_rate_t data_rate;
    ret = ism330bx_gy_data_rate_get(ctx, &data_rate);
    if (ret != 0)
    {
        LOG_ERR("Failed to get gyroscope data rate");
        return -EIO;
    }
    ism330bx_helper_print_gyro_odr(&data_rate);

    ism330bx_gy_self_test_t self_test;
    ret = ism330bx_gy_self_test_get(ctx, &self_test);
    if (ret != 0)
    {
        LOG_ERR("Failed to get gyroscope self test");
        return -EIO;
    }
    ism330bx_helper_print_gyro_self_test(&self_test);

    ism330bx_gy_full_scale_t full_scale;
    ret = ism330bx_gy_full_scale_get(ctx, &full_scale);
    if (ret != 0)
    {
        LOG_ERR("Failed to get gyroscope full scale");
        return ret;
    }
    ism330bx_helper_print_gyro_full_scale(&full_scale);

    LOG_DBG("Finished reading gyroscope config");
    return EXIT_SUCCESS;
}
