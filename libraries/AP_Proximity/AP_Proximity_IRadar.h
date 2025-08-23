/*
* Author:- Ayush Raman
* Copyright:- IoTechWorld Avigation Pvt Ltd.
*/
#pragma once
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include "AP_HAL_ChibiOS/IRadar_Backend.h"
#include "AP_Proximity_Filters.h"
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_WPNav/AC_WPNav.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAL_WITH_UAVCAN


#define PROXIMITY_RADAR_IR24_TIMEOUT_MS 500 // requests timeout after 0.2 seconds
#define PROXIMITY_SECTOR_WIDTH_DEG      45.0f //width of sector in degree.
#define PI 3.141592654f
#define MAX_SECTOR_TIMEOUT  150
#define MODE_LOITER   5
#define MODE_AUTO     3
#define MODE_BRAKE    17
#define RANGEFINDER_MAX_INSTANCES 10
#define BUFFER_SIZE 5
#define DISTANCE_THRESHOLD 5
#define ANGLE_THRESHOLD 20
#define TIMEOUT_THRESHOLD 1500

class AP_Proximity_IR24 : public AP_Proximity_Backend
{

public:


    AP_Proximity *_param_state = AP::proximity();
    // constructor
    AP_Proximity_IR24(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    typedef struct {
        float glitch_dis;
        float glitch_ang;
        uint32_t timestamp;
    } Reading;

    typedef struct {
        Reading buffer[BUFFER_SIZE];
        int count; 
    } GlitchFilter;

    GlitchFilter filterFront, filterBack;
    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 160.0f; }
    float distance_min() const override { return 0.20f; }

    float findDistance(double lat1, double lon1, double lat2, double lon2);
    float findBearing(double lat1, double lon1, double lat2, double lon2);

    void send_position_data(float latitude, float longitude, float curBearing, uint8_t min_pts, uint8_t epsilon){
             IR24_Backend *_ir24_backend = AP::__backend();
            _ir24_backend->send_position_data_backend(latitude, longitude, curBearing, min_pts, epsilon);
    }
    void initGlitchFilter(GlitchFilter *filter);
    bool isGlitch(GlitchFilter *filter, float distance, float angle);

private:
    //HAL_Semaphore _r21_sem;
    uint32_t _last_update_ms;   // system time of last RangeFinder reading
    uint8_t _last_sector = UINT8_MAX;

    uint32_t _last_sector_time[8];

    uint32_t now_ms; 
    uint32_t _last_position_update;
    uint32_t debugtime;
    uint32_t blk_rd;
    float last_dis_f = 0.0;
    float last_dis_b = 0.0;

    //uint32_t debugtime1;
    const float EARTH_RADIUS = 6371000.0f;
    // get a reading
    // distance returned in reading_cm
    bool get_reading();
    void check_inactive_sector(void);

};
#endif
#endif