#include "SCSCL.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

struct SERVO_STATE
{
    SERVO_STATE(u8 id) : ID(id) {}

    u8 ID                   = 0;
    u8 torque_switch        = 0;
    u16 goal_position       = 512; // middle position
    u16 present_position    = 0;
    s16 present_velocity    = 0;
    s16 present_load        = 0;
};

void SERVO_TASK(void * parameters);

class SERVO : public SCSCL
{
public:
    SERVO();
    void disable();
    void enable();
    void enableTorque();
    void disableTorque(); 
    void rotate(u8 servoID);
    void setStartPos(u8 servoID);
    void setMidPos(u8 servoID);
    void setEndPos(u8 servoID);
    int  setPosition(u8 servoID, u16 position, u16 speed = 0); // default maximum speed
    int  setPositionFast(u8 servoID, u16 position);                         // not thread-safe, to be deleted
    void setPosition12(u8 const servoIDs[], u16 const servoPositions[]);    // not thread-safe
    bool checkPosition(u8 servoID, u16 position, int accuracy);
    void setID(u8 servoID, u8 newID);

    bool isEnabled; 
    bool isTorqueEnabled; 

    /*** ASYNC API Work In Progress ***/

    // state of all servo
    SERVO_STATE state[12] {1,2,3,4,5,6,7,8,9,10,11,12};                     // hard-coded ID list

    // accessers of servo state
    void setPositionAsync(u8 servoID, u16 servoPosition);
    u16  getPositionAsync(u8 servoID);
    u16  getVelocityAsync(u8 servoID);
    u16  getLoadAsync(u8 servoID);

    // internals
    void sync_goal_position();

    // background servo bus service
    bool isRunning = true;
    TaskHandle_t task_handle = NULL;

    /*** ASYNC API Work In Progress ***/
};

#endif
