#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Iotech_Radar.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <ctype.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_IOTECH_RADAR::AP_RangeFinder_IOTECH_RADAR(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params):
    AP_RangeFinder_Backend(_state, _params)
{

}

/*
   detect if a uLanding rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_IOTECH_RADAR::detect()
{
    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_IOTECH_RADAR::get_reading(uint16_t &altitude_cm)
{

    IR24_Backend *_iradar_backend = AP::__backend();

    if (_iradar_backend->ir24_rfnd_available() == 0x01)
    {
        
        chMtxLock(_iradar_backend->get_ir24_mtx());
        if(!_iradar_backend->get_item_rfnd())
        {
            altitude_cm = altitude_bckp;
        }
        else{
            altitude_cm = _iradar_backend->get_item_rfnd();
            altitude_bckp = altitude_cm;
        }

            //if(AP_HAL::millis() - debugTime > 1000)
            ///{
                // gcs().send_text(MAV_SEVERITY_INFO, "alt: %0.2f", (float)altitude_cm/100);
            //    debugTime = AP_HAL::millis();
           // }
        // AP::logger().Write("IRT", "TimeUS,Alt", "QH",
        //                     AP_HAL::micros64(),
        //                     altitude_cm
        //                     );
        _iradar_backend->ir24_rfnd_reset_new_data_flag();
        chMtxUnlock(_iradar_backend->get_ir24_mtx());
        if((int8_t)params.enable_filter > 0){
            state.range_distance_cm = altitude_cm;
            apply_glitch_filter(altitude_cm);
        }
        return true;
    }
    else{
        return false;
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_IOTECH_RADAR::update(void){
    if (get_reading(state.distance_cm)) {
        state.last_reading_ms = AP_HAL::millis();
        // update range_valid state based on distance measured
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 2000) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

//glitch filter
void AP_RangeFinder_IOTECH_RADAR::apply_glitch_filter(uint16_t &reading_cm)
{
    uint16_t current_reading = reading_cm; //backup current reading

    //disarm state filter
    if(AP_Notify::flags.armed == false){
        check_flag = true;
        if(reading_cm<=(uint16_t)params.glitch_cm ){
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
            glitch_reading = reading_cm;
            return;
        }
        if(reading_cm >(uint16_t)params.glitch_cm ){
            reading_cm = (uint16_t)params.min_distance_cm;
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
            glitch_reading = reading_cm;
            return;
        }
    }

    //compare takeoff throttle
    if((AP_Notify::flags.armed == true) && (check_flag== true)){
        uint16_t my_ch1 = hal.rcout->read(1);
        if(my_ch1>(uint16_t)params.pwm_check){
            read_time_ms = AP_HAL::millis();
            check_flag = false;
        }
        else{
            if(reading_cm<=(uint16_t)params.glitch_cm ){
                if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
                glitch_reading = reading_cm;
                return;
            }
            if(reading_cm >(uint16_t)params.glitch_cm ){
                reading_cm = (uint16_t)params.min_distance_cm;
                if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
                glitch_reading = reading_cm;
                return;
            }
        }
    }

    if((AP_Notify::flags.armed == true) && (AP_HAL::millis() - read_time_ms <= (uint16_t)params.time_check)){
        
        if((glitch_reading - reading_cm) >= (uint16_t)params.glitch_cm ){
            //it means there is a positive glitch
            reading_cm = glitch_reading; //as of now we will send prev reading which is approved not be glitch will be sent to fc
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        }
        else if((reading_cm - glitch_reading) >= (uint16_t)params.glitch_cm){
            //it means we get a negative glitch
            reading_cm = glitch_reading; //as of now we will send prev reading which is approved not be glitch will be sent to fc
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        }
        else{
            //back up new glitch_reading and restore the original reading cm
            reading_cm = current_reading;
            glitch_reading = reading_cm;
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        }
        return;
    }

    if((glitch_reading - reading_cm) >= (uint16_t)params.glitch_cm ){
        //it means there is a positive glitch
        glitch_samples+= 1; //increment glitch sample
        reading_cm = glitch_reading; //as of now we will send prev reading which is approved not be glitch will be sent to fc
        if(abs(glitch_samples)!=(uint8_t)params.glitch_samples_num){
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        }
    }
    else if((reading_cm - glitch_reading) >= (uint16_t)params.glitch_cm){
        //it means we get a negative glitch
        glitch_samples-= 1; //decrement the glitch sample
        reading_cm = glitch_reading; //as of now we will send prev reading which is approved not be glitch will be sent to fc
        if(abs(glitch_samples)!=(uint8_t)params.glitch_samples_num){
            if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        }
    }
    else{
        //we don't get a glitch
        glitch_samples = 0; //reset glitch samples counter

        //back up new glitch_reading and restore the original reading cm
        reading_cm = current_reading;
        if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        glitch_reading = reading_cm;
    }

    if(abs(glitch_samples) >= (uint8_t)params.glitch_samples_num){
        glitch_samples = 0; //reset glitch samples counter

        //it means we get consistent glitch no need to change reading_cm as that is an actual data so restore the reading 
        reading_cm = current_reading;
        if(params.enable_mov == 1){apply_Mov_average(reading_cm);}
        glitch_reading = reading_cm;
    }

    return;
}
void AP_RangeFinder_IOTECH_RADAR::apply_Mov_average(uint16_t &reading_cm){

    if ((reading_cm > (int16_t)params.max_distance_cm) || (reading_cm < (int16_t)params.min_distance_cm)){
        return;
    }else{

        if(state.last_value_unstable){
            memset(mov_read,0,sizeof(mov_read));
            state.last_value_unstable = false;
        }

        uint16_t avg_value = 0;
        uint8_t avg_count = 0;
        uint8_t _sm_count = 24;
        if((uint8_t)params._sm_count<25){
            _sm_count = (uint8_t)params._sm_count;
        }
        if(Mov_sample>(_sm_count-1)){
            Mov_sample = 0;
        }
        mov_read[Mov_sample]= reading_cm;
        for(uint8_t i=0;i<_sm_count;i++){
            if(mov_read[i] != 0){
                avg_value = avg_value + mov_read[i];
                avg_count++;
            }
        }
        if(avg_count == 0){
            reading_cm = 0;
            return;
        }
        avg_value = avg_value/avg_count;
        reading_cm = avg_value;
        Mov_sample+=1;
        return;
    }
}
#endif
