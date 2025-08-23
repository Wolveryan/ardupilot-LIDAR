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

#pragma once
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// #define HAL_WITH_UAVCAN 1
#if HAL_WITH_UAVCAN
//#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "CANThread.h"
#include "CANClock.h"
#include "CANIface.h"
#include <AP_Math/AP_Math.h>

struct IR24_RouteItem {
    uint64_t utc_usec;
    uavcan::CanFrame frame;
    IR24_RouteItem() :
        utc_usec(0)
    {
    }
};

class IR24_Backend {

public:

    IR24_Backend() // mr72 queue store max 10 samples
    {
        _singleton = this;
    }

    IR24_Backend(const IR24_Backend &other) = delete;
    IR24_Backend &operator= (const IR24_Backend&) = delete;

    enum enum_IR24_CANID{
        IR24_front = 0xD1,
        IR24_rear  = 0xE1,
        IR24_rangefinder = 0xCF, 
    };

    static IR24_Backend *get_singleton() {
        return _singleton;
    } 

    struct data_frame{
        uint16_t _ir24_canId;
        Vector2l _ir24_coord;
    };
    // struct terrain_frame{
    //     uint16_t _ir24_ter_canId;
    //     uint16_t  _ir24_alt;
    // };

    bool ir24_get_ir24_sem_init() {return _ir24_sem_init;}
    void ir24_init_thread(void);
    void ir24_loop(void);

    void route_frame_to_ir24_backend(const uavcan::CanFrame& frame, uint64_t timestamp_usec);

    bool ir24_handleFrame(const uavcan::CanFrame& frame);
    bool ir24_available();
    void ir24_reset_new_data_flag();
    void send_position_data_backend(float latitude, float longitude, float curBearing, uint8_t min_pts, uint8_t epsilon);

    data_frame get_item();
    //terrain_frame get_terrain();
    uint16_t get_item_rfnd() {return alt; }
    uint32_t get_que_available();
    //uint32_t get_terrain_que_available();
    bool ir24_rfnd_available();
    void ir24_rfnd_reset_new_data_flag();

    binary_semaphore_t *get_ir24_sem() { return &_ir24_sem; }
    mutex_t *get_ir24_mtx() { return &_ir24_mtx; }
    ObjectBuffer<data_frame> *items;
    //ObjectBuffer<terrain_frame> *ter_items;
    binary_semaphore_t sem;
private:

    binary_semaphore_t _ir24_sem;
    mutex_t _ir24_mtx;
    IR24_RouteItem _ir24_item;
    bool _ir24_sem_init = false;
    // bool _data_pushed = false;
    bool _ir24_new_data = false;
    bool _ir24_rfnd_new_data = false;
    uint8_t  _tot_frame_avai = 0;
    static IR24_Backend *_singleton;
    uint16_t alt = 0;
    //uint32_t debugtime;

};
namespace AP{
    IR24_Backend *__backend();
};

#endif
#endif