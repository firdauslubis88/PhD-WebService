#include "PTZControl.h"
#include <iostream>
#include <omp.h>
//#include "Poco/TaskNotification.h"

static double TIME_THRESHOLD = 1;
PTZControl::PTZControl(std::string port)
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	index = 0;
	dxl_comm_result = COMM_TX_FAIL;             // Communication result
												//dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

	dxl_error = 0;                          // Dynamixel error
	dxl_present_position_pan = 0;               // Present position
	dxl_present_position_tilt = 0;
	// Open port
	if (portHandler->openPort())
	{
//		printf("Succeeded to open the port!\n");
		opened = true;
	}
	else
	{
//		printf("Failed to open the port!\n");
		opened = false;
		return;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
//		printf("Succeeded to change the baudrate!\n");
		opened = true;
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		opened = false;
		return;
	}

	// Enable Dynamixel Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
		opened = false;
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
		opened = false;
	}
	else
	{
//		printf("Dynamixel has been successfully connected \n");
		opened = true;
	}
}


PTZControl::~PTZControl()
{
}


int PTZControl::SetPanning(float input) {
	if (!opened)
	{
		return -999;
	}
	uint8_t dxl_error;
	int dxl_goal_position_pan = static_cast<int>((input + 180.0f) * 4096.0f / 360.0f);
	int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION, dxl_goal_position_pan, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	return dxl_comm_result;
}

float PTZControl::GetPanning() {
	double start_time = omp_get_wtime(), end_time;
	if (!opened)
	{
		return -999;
	}
	uint8_t dxl_error;
	int dxl_present_position_pan;
	int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION, (uint32_t*)&dxl_goal_position_pan, &dxl_error);

	do
	{
		dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_pan, &dxl_error);

		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
		}
		end_time = omp_get_wtime();
	} while ((abs(dxl_goal_position_pan - dxl_present_position_pan) > DXL_MOVING_STATUS_THRESHOLD) && ((end_time - start_time)< TIME_THRESHOLD));

	dxl_present_position_pan = (dxl_present_position_pan * 360 / 4096) - 180;
	return static_cast<float>(dxl_present_position_pan);
}

int PTZControl::SetTilting(float input) {
	if (!opened)
	{
		return -999;
	}
	uint8_t dxl_error;
	int dxl_goal_position_tilt = static_cast<int>((input + 180.0f) * 4096.0f / 360.0f);
	int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position_tilt, &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	return 0;
}

float PTZControl::GetTilting() {
	double start_time = omp_get_wtime(), end_time;
	if (!opened)
	{
		return -999;
	}
	uint8_t dxl_error;
	int dxl_present_position_tilt;
	int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, (uint32_t*)&dxl_goal_position_tilt, &dxl_error);

	do
	{
		dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_tilt, &dxl_error);
	
		if (dxl_comm_result != COMM_SUCCESS)
		{
		packetHandler->printTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
		packetHandler->printRxPacketError(dxl_error);
		}
		end_time = omp_get_wtime();
	} while ((abs(dxl_goal_position_tilt - dxl_present_position_tilt) > DXL_MOVING_STATUS_THRESHOLD) && ((end_time - start_time) < TIME_THRESHOLD));
	dxl_present_position_tilt = (dxl_present_position_tilt * 360 / 4096) - 180;
	return static_cast<float>(dxl_present_position_tilt);
}

int PTZControl::getch() {
	if (!opened)
	{
		return -999;
	}
	return _getch();
}

int PTZControl::kbhit(void) {
	if (!opened)
	{
		return -999;
	}
	return _kbhit();
}