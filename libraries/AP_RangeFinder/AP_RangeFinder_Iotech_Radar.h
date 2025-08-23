#pragma once
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include "AP_HAL_ChibiOS/IRadar_Backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

class AP_RangeFinder_IOTECH_RADAR : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_IOTECH_RADAR(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);
    void apply_Mov_average(uint16_t &reading_cm);
    void apply_glitch_filter(uint16_t &reading_cm);

    uint16_t prev_dist_cm = 0;
    uint32_t read_time_ms;
    bool check_flag = false;
    uint8_t Mov_Avg_Filter_Coeff = 67;
    uint32_t debugTime = 0;
    uint16_t glitch_reading;
    int8_t glitch_samples;
    uint16_t altitude_bckp = 0;

    uint8_t Mov_sample = 0;
    uint16_t mov_read[25];
};
#endif

