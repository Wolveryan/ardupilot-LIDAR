/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   Author:- Ayush Raman
   Copyright:- IoTechWorld Avigation Pvt Ltd.
 */


#include "AP_Proximity_IRadar.h"
//#define CONFIG_HAL_BOARD HAL_BOARD_CHIBIOS
//#define HAL_WITH_UAVCAN 1
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAL_WITH_UAVCAN
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_Proximity.h"
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_RangeFinder/RangeFinder.h>

const float OA_LOW_SPEED_SQUARED = (0.2f * 0.2f);    // when ground course is below this speed squared, vehicle's heading will be used

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_Proximity_IR24::AP_Proximity_IR24(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{  
    initGlitchFilter(&filterFront);
    initGlitchFilter(&filterBack);
}


bool AP_Proximity_IR24::detect()
{
    return true;
}

//Initialize count of valid readings
void AP_Proximity_IR24::initGlitchFilter(GlitchFilter *filter) {  
    
    filter->count = 0;
    //gcs().send_text(MAV_SEVERITY_INFO,"Init Glitch");

    for(int i = 0; i < BUFFER_SIZE; i++) {
        filter->buffer[i].glitch_dis = 0.0;
        filter->buffer[i].glitch_ang = 0.0;
        filter->buffer[i].timestamp = 0;
    }
}
// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_Proximity_IR24::get_reading()
{
    IR24_Backend *_ir24_backend = AP::__backend();
    uint8_t sector = 0;
    Vector2l coordi;
    float lat;
    float lon;
    double latd;
    double lond;
    double curlatd;
    double curlond;
    float curlat;
    float curlon;
    float distance = 0;
    float angle_degree = 0;
    uint16_t canId;
    float _ir24_max_fov_range = 0;
    float _ir24_min_fov_range = 0;
    float _ir24_mid_fov_range1 = 0;
    float _ir24_mid_fov_range2 = 0;
    float _true_angle = 0;
    // uint16_t alt = 0;
    //uint32_t dis_bckp = 0;
    //uint8_t ang_bckp = 0;

    // Call MR72 Can Backend to fetch the values
    if(_ir24_backend->ir24_available() == 0x01)
    {
        //take mutex from MR72_BACKEND
        chMtxLock(_ir24_backend->get_ir24_mtx());
        while(_ir24_backend->get_que_available()){

            Location current_loc;
            if (!AP::ahrs().get_position(current_loc)){
                return false;
            }

            bool use_front = false;
            bool use_back = false;
            Vector2f ground_speed_vec = AP::ahrs().groundspeed_vector();
            uint8_t flight_mode = AP_Notify::flags.flight_mode;
            if ((ground_speed_vec.length_squared() < OA_LOW_SPEED_SQUARED) || ( flight_mode == MODE_LOITER) || (flight_mode == MODE_BRAKE) || (_param_state->_enable_scope == 0)) {
                use_front = true;
                use_back = true;
            } else {
                float ground_course_deg = wrap_360(degrees(ground_speed_vec.angle()) + 360);
                ground_course_deg = wrap_360(ground_course_deg - (AP::ahrs().yaw_sensor * 0.01f));
                if ((ground_course_deg > 90.f) && (ground_course_deg < 270.0f)) {
                    use_back = true;
                } else {
                    use_front = true;
                }

            }

            if(_param_state->_att_enable) {
                RangeFinder *rangefinder = RangeFinder::get_singleton();

                for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
                    if (rangefinder == nullptr) {
                        break;
                    }
                    const AP_RangeFinder_Backend *s = rangefinder->get_backend(i);
                    if (s == nullptr) {
                        continue;
                    }
                    // alt = s->distance_cm();
                }
            }


            IR24_Backend::data_frame _tdf = _ir24_backend->get_item();
            canId = _tdf._ir24_canId;
            coordi = _tdf._ir24_coord;
            if(!coordi.x || !coordi.y)
            {
                continue;
            }
            latd = (coordi.x * 0.000001);
            lond = (coordi.y * 0.000001);
            lat = (float)latd;
            lon = (float)lond;

             if(_param_state->_att_enable == 2) {
                curlatd = 22.463534;
                curlond = 77.434532;
             }
             else{
                curlatd = current_loc.lat * 0.0000001;
                curlond = current_loc.lng * 0.0000001;
             }

            curlat = (float)curlatd;
            curlon = (float)curlond;
            distance = findDistance(curlatd, curlond, latd, lond);
            angle_degree = findBearing(curlatd, curlond, latd, lond);
            
            float curBearing =  (AP::ahrs().yaw_sensor * 0.01f);
            if(canId == IR24_Backend::enum_IR24_CANID::IR24_rear)
            {
                 curBearing += 180.0f; 
                 if(curBearing >= 360.0f)
                 {
                     curBearing -= 360.0f;
                 }
            }
            float relative_angle = angle_degree - curBearing;
            if(relative_angle < 0.0)
            {
                relative_angle += 360.0;
            }
            
            angle_degree = relative_angle;

            // if(_param_state->_att_enable == 1){
            //     if((alt != 0 && alt < 200) && (AP_Notify::flags.armed == true)){
            //         distance = 0.0f;
            //         angle_degree = 0.0f;
            //     }
            // }
            if(_param_state->_enable_filter)
            {
                bool glitch = false;
                if(canId == IR24_Backend::enum_IR24_CANID::IR24_front)
                {
                    glitch = isGlitch(&filterFront, distance, angle_degree);
                }
                else if(canId == IR24_Backend::enum_IR24_CANID:: IR24_rear) 
                {
                    glitch = isGlitch(&filterBack, distance, angle_degree);
                }
                
                if(glitch)
                {
                    distance = 0.0f;
                    angle_degree = 0.0f;
                }
            }

            if(_param_state->_enable_filter == 2)
            {
                if(canId == IR24_Backend::enum_IR24_CANID::IR24_front){
                    if(!last_dis_f){
                        last_dis_f = distance;
                    }
                    if(distance > 23.0 && distance < 26.0)
                    {
                        if(last_dis_f < distance)
                        {
                            distance = 0.0f;
                            angle_degree = 0.0f;
                        }
                        else if(last_dis_f > 24.0 && last_dis_f < 26.0)
                        {
                            distance = 0.0f;
                            angle_degree = 0.0f;
                        }
                        else{
                            last_dis_f = distance;
                        }
                    }
                    else{
                        last_dis_f = distance;
                    }
                }
                if(canId == IR24_Backend::enum_IR24_CANID::IR24_rear){
                    if(!last_dis_b)
                    {
                        last_dis_b = distance;
                    }
                    if(distance > 23.0 && distance < 26.0)
                    {
                        if(last_dis_b < distance)
                        {
                            distance = 0.0f;
                            angle_degree = 0.0f;
                        }
                        else if(last_dis_b > 24.0 && last_dis_b < 26.0)
                        {
                            distance = 0.0f;
                            angle_degree = 0.0f;
                        }
                        else{
                            last_dis_b = distance;
                        }
                    }
                    else{
                        last_dis_b = distance;
                    }
                }
            }
            else if(_param_state->_enable_filter == 4)
            {
                if(distance > 23.0 && distance < 26.0){
                    distance = 0.0f;
                    angle_degree = 0.0f;
                }

            }

            float _log_dis = distance;
            float _log_angle= angle_degree;
            float _log_dis_f = 0;
            float _log_angle_f = 0;

            //check for recieved can id
            if(canId == IR24_Backend::enum_IR24_CANID::IR24_front)
            {
                _log_dis_f = distance;
                _log_angle_f = angle_degree;
                //find sector from angle
                sector = convert_angle_to_sector(angle_degree); 

                //update fov limits for front radar   
                _ir24_max_fov_range = _param_state->_fov_limit;  
                _ir24_min_fov_range = 360.0f - _param_state->_fov_limit;
                _ir24_mid_fov_range1 = 360.0f;
                _ir24_mid_fov_range2 = 0.0f;  
                if (!use_front) {
                    distance = 0.0f;
                    angle_degree = 0.0f;
                }
            }
            else if(canId == IR24_Backend::enum_IR24_CANID::IR24_rear)
            {
                 _log_dis_f = distance;
                 _log_angle_f = angle_degree;

                //add 180 degree to angle for rear radar
                angle_degree = angle_degree + 180.0f;
                //wrap angle in 360 degree
                angle_degree = wrap_360(angle_degree);
                //find sector from angle
                sector = convert_angle_to_sector(angle_degree);
                //update fov limits from rear radar
                _ir24_max_fov_range = 180.0f + _param_state->_fov_limit;  
                _ir24_min_fov_range = 180.0f - _param_state->_fov_limit;  
                _ir24_mid_fov_range1 = 180.0f;     
                _ir24_mid_fov_range2 = 180.0f;  
                if (!use_back) {
                    // if object is out of range , set distance and angle out of range and sector valid false
                    distance = 0.0f;
                    angle_degree = 0.0f;
                }
            }
            else {
                //if some how no can id is found
                chMtxUnlock(_ir24_backend->get_ir24_mtx()); 
                return false;
            }

            //check for new sector
            if(sector != _last_sector)
            {
                //update boundary used for avoidance
                if (_last_sector != UINT8_MAX) {
                    update_boundary_for_sector(_last_sector, false);
                }
                // init for new sector
                _last_sector = sector;
                _last_sector_time[sector] = AP_HAL::millis();
            }
            else
            {   //upadate time for active sector
                _last_sector_time[sector] = AP_HAL::millis();
            }

            //check for inactive sector
            check_inactive_sector();

            //check object distance in valid range NOTE - _param_state->_max_dist and _param_state->_min_dist 
            //will be in meters.
            if(((distance > _param_state->_min_dist) && (distance < _param_state->_max_dist)) && 
            ((angle_degree >= _ir24_min_fov_range && angle_degree < _ir24_mid_fov_range1)|| 
            (angle_degree >= _ir24_mid_fov_range2 && angle_degree <= _ir24_max_fov_range)))
            {
                //object is in valid range for avoidance
                _distance[sector] = distance; 
                _distance_valid[sector] = true;
                _angle[sector] = angle_degree;
                _true_angle = angle_degree;
                Location obstacle_location = Location{(coordi.x * 10) , (coordi.y * 10), 0,Location::AltFrame::ABSOLUTE};
                database_lat_lng_push(obstacle_location, distance);
            }
            else
            {
                // if object is out of range , set distance and angle out of range and sector valid false
                _distance[sector] = 0.0f;
                _angle[sector] = 0.0f;
                _true_angle = 0.0f;
                _distance_valid[sector] = false;
            }

           //log parameters
            if(canId == IR24_Backend::enum_IR24_CANID::IR24_front){

                AP::logger().Write("IRF", "TimeUS,Dis,ang,Tang,Rd,Ra,fd,fa,lat,lon,UF,LA,LN", "QfffffffffBff",
                                    AP_HAL::micros64(),
                                    _distance[sector],
                                    _angle[sector],
                                    _true_angle,
                                    _log_dis,
                                    wrap_360(_log_angle),
                                    _log_dis_f,
                                    wrap_360(_log_angle_f),
                                    lat,
                                    lon,
                                    use_front,
                                    curlat,
                                    curlon
                );
            }
            else{
                AP::logger().Write("IRB", "TimeUS,Dis,ang,Tang,Rd,Ra,fd,fa,lat,lon,UB,LA,LN", "QfffffffffBff",
                                    AP_HAL::micros64(),
                                    _distance[sector],
                                    _angle[sector],
                                    _true_angle,
                                    _log_dis,
                                    wrap_360(_log_angle),
                                    _log_dis_f,
                                    wrap_360(_log_angle_f),
                                    lat,
                                    lon,
                                    use_back,
                                    curlat,
                                    curlon
                );
            }

            //Update databace for avoidance, register data from radar which is in the drone heading.
            update_boundary_for_sector(sector, false);

        }
        //reset data flag from IR24 BACKEND
        _ir24_backend->ir24_reset_new_data_flag();
        //give mutex to IR24 BACKEND
        chMtxUnlock(_ir24_backend->get_ir24_mtx()); 
        return true; 
    }
    // no readings so return false
    return false;
}


void AP_Proximity_IR24::check_inactive_sector(void)
{
    //check all inactive sector and clear that sector
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
        if (_distance_valid[i]) {
            if((AP_HAL::millis() - _last_sector_time[i]) >= MAX_SECTOR_TIMEOUT)
            {
                // clear that inactive sector
                _distance[i] = 0.0f;
                _angle[i] = 0.0f;
                _distance_valid[i] = false;
            }
        }
    }
}

//Scouting glitch
bool AP_Proximity_IR24::isGlitch(GlitchFilter *filter, float distance, float angle_degree)
{
    for(int i = 0; i < BUFFER_SIZE; i++) {
        if((fabs(distance - filter->buffer[i].glitch_dis) <= DISTANCE_THRESHOLD) && (fabs(angle_degree - filter->buffer[i].glitch_ang) <= ANGLE_THRESHOLD) && (fabs(AP_HAL::millis() - filter->buffer[i].timestamp) <= TIMEOUT_THRESHOLD)){
            return 0; //Not a Glitch 
        }
    }

    //fill the buffer until it's full
    if(filter->count < BUFFER_SIZE) {
        filter->buffer[filter->count].glitch_dis = distance;
        filter->buffer[filter->count].glitch_ang = angle_degree;
        filter->buffer[filter->count].timestamp = AP_HAL::millis();
        filter->count++;
        return 0;
    } else {
        //Replace the oldest reading in the buffer
        filter->buffer[filter->count % 5].glitch_dis = distance;
        filter->buffer[filter->count % 5].glitch_ang = angle_degree;
        filter->buffer[filter->count % 5].timestamp = AP_HAL::millis();
        filter->count++;
        return 1; //Glitch detected
    }
}
/* 
   update the state of the sensor
*/
void AP_Proximity_IR24::update(void)
{

    Location currentLoc;
    float curBearing;
    bool use_loc = false;
    //Get current position
    if (AP::ahrs().get_position(currentLoc)) {
        use_loc = true;
    }
    //Get current position
    curBearing = AP::ahrs().yaw_sensor * 0.01f;
    float lat = 0.0f;
    float lon = 0.0f;

    if(_param_state->_att_enable == 2){
        lat = 22.463534;
        lon = 77.434532;
    } else if (use_loc){
        lat = currentLoc.lat * 0.0000001f;
        lon = currentLoc.lng * 0.0000001f;
    }
    uint8_t min_pts = _param_state->_min_pts;
    uint8_t epsilon = _param_state->_epsilon;
    now_ms = AP_HAL::millis();
    
    if(get_reading()){
        // update range_valid state based on distance measured
       _last_update_ms = now_ms;
    }
     // check for timeout and set health status
    if ((_last_update_ms == 0) || (now_ms - _last_update_ms > PROXIMITY_RADAR_IR24_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
    //clear inactive sector
    if(now_ms - _last_update_ms > PROXIMITY_RADAR_IR24_TIMEOUT_MS)
    {
        check_inactive_sector();
    }

    float pitch = degrees(AP::ahrs().pitch);
    //when pitch angle cross 10 degrees, pause communication with radar
    if(pitch > 10.0f || pitch < -10.0f)
    {
        //resume radar data communication after 200ms, once the position comtroller has stablilized the drone
        if(blk_rd - AP_HAL::millis() < 200)
        {
            _last_position_update = AP_HAL::millis();
        }
    }
    else{
        blk_rd = AP_HAL::millis();
    }
    
    //send position data to the radar
    uint32_t now = AP_HAL::millis();
    if(now - _last_position_update > 30){
        send_position_data((float)lat, (float)lon, (float)curBearing, (uint8_t)min_pts, (uint8_t)epsilon);
        _last_position_update = now;
    }
}
/*
    Calculate distance from latitude & longitude
*/
float AP_Proximity_IR24::findDistance(double lat1, double lon1, double lat2, double lon2)
{
    double lat1Rad = lat1 * M_PI / 180.0f;
    double lon1Rad = lon1 * M_PI / 180.0f;
    double lat2Rad = lat2 * M_PI / 180.0f;
    double lon2Rad = lon2 * M_PI / 180.0f;

    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    double a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) + cosf(lat1Rad) * cosf(lat2Rad) * sinf(dLon / 2.0f) * sinf(dLon / 2.0f);
    double c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    float distance = (EARTH_RADIUS * c);

    return distance;
}

/*
    Find bearing of the target from the latitude and longitude data of the target
*/
float AP_Proximity_IR24::findBearing(double lat1, double lon1, double lat2, double lon2)
{
    //Convert latitude and longitude from degrees to radians
    double lat1Rad = lat1 * M_PI / 180.0f;
    double lon1Rad = lon1 * M_PI / 180.0f;
    double lat2Rad = lat2 * M_PI / 180.0f;
    double lon2Rad = lon2 * M_PI / 180.0f;

    // Calculate differences bearing
    double dLon = lon2Rad - lon1Rad;

    //Calculate initial bearing
    double x = sinf(dLon) * cosf(lat2Rad);
    double y = cosf(lat1Rad) * sinf(lat2Rad) - sinf(lat1Rad) * cosf(lat2Rad) * cosf(dLon);
    double bearing = atan2f(x,y);

    //Convert bearing from radians to degrees
    bearing = fmodf((bearing * 180.0f / M_PI + 360.0f), 360.0f);

    return (float)bearing;
}
#endif
#endif