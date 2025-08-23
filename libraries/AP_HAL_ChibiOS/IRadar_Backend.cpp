/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author:- Ayush Raman
 * Copyright:- IoTechWorld Avigation Pvt Ltd.
 */

#include "IRadar_Backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAL_WITH_UAVCAN
#include "AP_HAL_ChibiOS/CANSerialRouter.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

IR24_Backend *IR24_Backend::_singleton;

void IR24_Backend::ir24_init_thread(void)
{
    //init a binary semaphore in taken state
    chBSemObjectInit(&_ir24_sem, true);
    chBSemObjectInit(&sem, false);
    _ir24_sem_init = true;

    chMtxObjectInit(&_ir24_mtx);

    items = new ObjectBuffer<data_frame>(10);
    //ter_items = new ObjectBuffer<terrain_frame>(10);

    //start a thread for synch. with isr
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&IR24_Backend::ir24_loop, void), "_ir24_Router", 4096, AP_HAL::Scheduler::PRIORITY_CAN, -1)) {
        return;
    }
}

void IR24_Backend::ir24_loop(void)
{
    IR24_RouteItem it;

    while(true)
    {
        msg_t msg = chBSemWaitTimeout(&_ir24_sem, (sysinterval_t)TIME_MS2I(100));

        if(msg == MSG_TIMEOUT)
        {
            continue;
        }
        if(msg == MSG_OK)
        {
            while(_tot_frame_avai != 0)
            {
                it.frame = _ir24_item.frame;                
                it.utc_usec = _ir24_item.utc_usec;          

                chMtxLock(&_ir24_mtx);                 
                ir24_handleFrame(it.frame);
                chMtxUnlock(&_ir24_mtx);
                _tot_frame_avai--;
                hal.scheduler->delay_microseconds(100);
            }             
        }
        hal.scheduler->delay_microseconds(100);
    }
}


void IR24_Backend::route_frame_to_ir24_backend(const uavcan::CanFrame& frame, uint64_t timestamp_usec){

    IR24_RouteItem it;
    it.frame = frame;

    it.utc_usec = timestamp_usec;
    _ir24_item.frame = it.frame;
    _ir24_item.utc_usec = it.utc_usec;
    _tot_frame_avai++;
}

bool IR24_Backend::ir24_handleFrame(const uavcan::CanFrame& frame)
{

    IR24_RouteItem it;
    it.frame = frame;
    uint32_t lat = 0;
    uint32_t lon = 0;           
    uint32_t altitude = 0;
    Vector2l _coord;

    // data pipeline

    uint16_t pk_pri =  (it.frame.id & 0xFF) | (it.frame.id & (0xFF00));
   // gcs().send_text(MAV_SEVERITY_INFO, "FrameId: %x", pk_pri);
    if(pk_pri == 0xD1 || pk_pri == 0xE1){
        data_frame _df;
        _df._ir24_canId = pk_pri;

       //gcs().send_text(MAV_SEVERITY_INFO, "%X %X %X %X",it.frame.data[0], it.frame.data[1], it.frame.data[2], it.frame.data[3]);
       //gcs().send_text(MAV_SEVERITY_INFO, "%X %X %X %X",it.frame.data[4], it.frame.data[5], it.frame.data[6], it.frame.data[7]);
        _coord.x = 0;
        _coord.y = 0;

        lat |= (it.frame.data[0]);
        lat |= (it.frame.data[1] << 8);
        lat |= (it.frame.data[2] << 16);
        lat |= (it.frame.data[3] << 24);
        lon |= (it.frame.data[4]);
        lon |= (it.frame.data[5] << 8);
        lon |= (it.frame.data[6] << 16);
        lon |= (it.frame.data[7] << 24);

        _coord.x = (int32_t)lat;
        _coord.y = (int32_t)lon;
        _df._ir24_coord.x = _coord.x;
        _df._ir24_coord.y = _coord.y;

        bool pushed = false;
        hal.scheduler->delay(10);
        msg_t msg = chBSemWaitTimeout(&sem, TIME_IMMEDIATE);
        if(msg==MSG_OK){

            items->push(_df);
            pushed = true;
            _ir24_new_data = true;
            chBSemSignalI(&sem);
        }
        return pushed;
    }
    if(pk_pri == 0xCF)
    {
        altitude |= (it.frame.data[0]);
        altitude |= (it.frame.data[1] << 8);
        altitude |= (it.frame.data[2] << 16);
        altitude |= (it.frame.data[3] << 24);

        alt = (uint16_t)(altitude/10000);
        _ir24_rfnd_new_data = true;
        return true;
    }
    return false;
}

void IR24_Backend::send_position_data_backend(float latitude, float longitude, float curBearing, uint8_t min_pts, uint8_t epsilon){

    int32_t lat = latitude * 1000000;
    int32_t lon = longitude * 1000000;
    // Generate the packet
    uint8_t data[8];
    uint8_t heading[4];
    uint16_t bear = (float)curBearing * 100;

    data[0] = ((int32_t)lat >> 24) & 0xFF;
    data[1] = ((int32_t)lat >> 16)& 0xFF;
    data[2] = ((int32_t)lat >> 8) & 0xFF;
    data[3] = ((int32_t)lat & 0xFF);
    data[4] = ((int32_t)lon >> 24) & 0xFF;
    data[5] = ((int32_t)lon >> 16) & 0xFF;
    data[6] = ((int32_t)lon >> 8) & 0xFF;
    data[7] = ((int32_t)lon & 0xFF); 

    heading[0] = (bear >> 8) & 0xFF;
    heading[1] = bear & 0xFF;
    heading[2] = min_pts & 0xFF;
    heading[3] = epsilon & 0xFF;

    //data[9] = 0x5A;
    //gcs().send_text(MAV_SEVERITY_INFO, "%X %X %X %X %X %X %X %X", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    //gcs().send_text(MAV_SEVERITY_INFO, "%X %X %X %X", heading[0], heading[1], heading[2], heading[3]);

    // Generate Extended CAN Packet
    uavcan::CanFrame frame1 = uavcan::CanFrame((uint32_t) 0xAFF | frame1.FlagEFF, data, (uint8_t)8);
    uavcan::CanFrame frame2 = uavcan::CanFrame((uint32_t) 0xAEF | frame2.FlagEFF, heading, (uint8_t)4);
    uint8_t drv = 0;
    uint8_t intf = 0;
    // bool success_f = false;
    // Broadcast to All Available Drivers and Interfaces as R21 packets operate irrespective of can protocol
    for(int i =0; i <MAX_NUMBER_OF_CAN_DRIVERS; i++){
        if(hal.can_mgr[i] != nullptr){
            drv++;
            for(int j =0; j <MAX_NUMBER_OF_CAN_INTERFACES; j++){
                if(hal.can_mgr[i]->get_driver()->getIface(j) != nullptr){
                    intf++;
                    bool success = hal.can_mgr[i]->get_driver()->getIface(j)->send(frame1, uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 1000), 0);
                    if(!success){
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "AWR Correction Packet Error");
                        continue;
                    }
                    else{
                        // success_f = true;
                    }

                    success = hal.can_mgr[i]->get_driver()->getIface(j)->send(frame2, uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 1000), 0);
                    if(!success){
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "AWR Correction Packet Error");
                    }
                }
            }
        }   
    }
    // AP::logger().Write("IRC","TimeUS,lt,ln,br,d,i,su","QfffIII",
    //                             AP_HAL::micros64(),
    //                             latitude,
    //                             longitude,
    //                             curBearing,
    //                             drv,
    //                             intf,
    //                             success_f
    //                             );
}

IR24_Backend::data_frame IR24_Backend::get_item(){
    data_frame _l;

    msg_t msg = chBSemWaitTimeout(&sem, TIME_IMMEDIATE);
    if(msg==MSG_OK){
        items->pop(_l);
        chBSemSignalI(&sem);
    }

    return _l;
}

// IR24_Backend::terrain_frame IR24_Backend::get_terrain(){
//     terrain_frame _t;
//     msg_t msg = chBSemWaitTimeout(&sem, TIME_IMMEDIATE);
//     if (msg==MSG_OK)
//     {
//         ter_items->pop(_t);
//         chBSemSignalI(&sem);
//         /* code */
//     }
//     return _t;
// }

uint32_t IR24_Backend::get_que_available(){
    uint32_t _l = 0;
    {
        msg_t msg = chBSemWaitTimeout(&sem, TIME_IMMEDIATE);
        if(msg==MSG_OK){
        _l = items->available();
        chBSemSignalI(&sem);
        }
    }
    return _l;
}

// uint32_t IR24_Backend::get_terrain_que_available(){
//     uint32_t _t = 0;
//     msg_t msg = chBSemWaitTimeout(&sem, TIME_IMMEDIATE);
//     if(msg==MSG_OK)
//     {
//         _t = ter_items->available();
//         chBSemSignalI(&sem);
//     }
//     return _t;
// }

bool IR24_Backend::ir24_available(){
    chMtxLock(&_ir24_mtx);
    bool ret = _ir24_new_data;
    chMtxUnlock(&_ir24_mtx);
    return(ret);
}

bool IR24_Backend::ir24_rfnd_available(){
   // chMtxLock(&_ir24_mtx);
    bool ret = _ir24_rfnd_new_data;
    //chMtxUnlock(&_ir24_mtx);
    return(ret);
}

void IR24_Backend::ir24_reset_new_data_flag(){
    _ir24_new_data = false;
}

void IR24_Backend::ir24_rfnd_reset_new_data_flag(){
    _ir24_rfnd_new_data = false;
}

namespace AP{
    IR24_Backend *__backend(){
        return IR24_Backend::get_singleton();
    }
}
#endif
#endif