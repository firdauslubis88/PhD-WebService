#pragma once

#include <conio.h>

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"	// Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESENT_VELOCITY       128


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          2                   // Dynamixel ID: 1 - pan
#define DXL_ID2                          1                   // Dynamixel ID: 2 - tilt
#define BAUDRATE                        57600

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     5                  // Dynamixel moving status threshold

class PTZControl
{
public:
	PTZControl(std::string port);
	~PTZControl();

	int SetPanning(float input);
	int SetTilting(float input);

	float GetPanning();
	float GetTilting();

	bool isOpened() { return this->opened; };

private:
	dynamixel::PortHandler *portHandler = NULL;
	dynamixel::PacketHandler *packetHandler = NULL;

	bool alreadyUsed = false;
	int index;
	int dxl_comm_result;
	int dxl_goal_position_pan;
	int dxl_goal_position_tilt;
	int newZoom;
	long min, max, SteppingDelta, currentValue, flags, defaultValue;

	uint8_t dxl_error;
	int32_t dxl_present_position_pan;
	int32_t dxl_present_position_tilt;

	int getch();
	int kbhit(void);

	int pPanControl = 1, iPanControl = 0, dPanControl = 0;
	int pan_error_accumulation = 0, pan_prev_error = 0;
	int pTiltControl = 1, iTiltControl = 0, dTiltControl = 0;
	int tilt_error_accumulation = 0, tilt_prev_error = 0;
	bool opened;
};