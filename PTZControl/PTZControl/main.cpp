#include <iostream>
#include "PTZControl.h"

const float PAN_UPPER_ANGLE_LIMIT = 60;
const float PAN_LOWER_ANGLE_LIMIT = -55;
const float TILT_UPPER_ANGLE_LIMIT = 45;
const float TILT_LOWER_ANGLE_LIMIT = -90;

//USAGE: PTZControl --> 1 arguments, it will reset the camera into 0 0 position. It will return [pan, angle] current camera position
//USAGE: PTZControl --> 2 arguments, it will get [pan, angle] current camera position. It will not set the camera position
//USAGE: PTZControl --> 3 arguments, it will set the camera argv[1] direction into argv[2] degree (not radian). It will return [pan, angle] current camera position
//USAGE: PTZControl --> 5 arguments, it will set the camera argv[1] direction into argv[2] degree (not radian) and camera argv[3] direction into argv[4] degree (not radian). It will return [pan, angle] current camera position
//NOTES: The output is printed to cout (stdoutput). 
//NOTES: This application (node js c++ automation) is specifically designed for node js web service applications!
//NOTES: Exceeded angle will be cut into their corresponding direction upper and lower angle limits
//NOTES: Giving wrong argv value (not 'pan'/'tilt' in argv [1]/[3]) will result in wrong return [-999/'pan' -999/'tilt'] and unintended direction angle position!!!
//NOTES: Failure to connect to PTZ camera will return [-999, -999] values

int main(int argc, char** argv) {
//	std::cout << "total args" << argc << std::endl;
//	std::cout << argv[0] << std::endl;
//	std::cout << argv[1] << std::endl;
	PTZControl ptz("COM2");
	if (!ptz.isOpened())
	{
		std::cout << -999 << std::endl;
		std::cout << -999;
		return -1;
	}
	if (argc == 2)
	{
		std::cout << ptz.GetPanning() << std::endl;
		std::cout << ptz.GetTilting();
		return 999;
	}
	if (argc == 1)
	{
		ptz.SetPanning(0);
		ptz.SetTilting(0);
	}
	else
	{
		char* direction = argv[1];
		float angle = 0;
		sscanf_s(argv[2], "%f", &angle);
		if (strcmp(direction, "pan") == 0)
		{
			if (angle > PAN_UPPER_ANGLE_LIMIT )
			{
				angle = PAN_UPPER_ANGLE_LIMIT;
			}
			else if (angle < PAN_LOWER_ANGLE_LIMIT)
			{
				angle = PAN_LOWER_ANGLE_LIMIT;
			}
			ptz.SetPanning(angle);
		}
		else if (strcmp(direction, "tilt") == 0)
		{
			if (angle > TILT_UPPER_ANGLE_LIMIT)
			{
				angle = TILT_UPPER_ANGLE_LIMIT;
			}
			else if (angle < TILT_LOWER_ANGLE_LIMIT)
			{
				angle = TILT_LOWER_ANGLE_LIMIT;
			}
			ptz.SetTilting(angle);
		}
		else
		{
			std::cout << -999 << std::endl;
			std::cout << -999;
			return -1;
		}
		if (argc == 5)
		{
			char* direction = argv[3];
			float angle = 0;
			sscanf_s(argv[4], "%f", &angle);
			if (strcmp(direction, "pan") == 0)
			{
				if (angle > PAN_UPPER_ANGLE_LIMIT)
				{
					angle = PAN_UPPER_ANGLE_LIMIT;
				}
				else if (angle < PAN_LOWER_ANGLE_LIMIT)
				{
					angle = PAN_LOWER_ANGLE_LIMIT;
				}
				ptz.SetPanning(angle);
			}
			else if (strcmp(direction, "tilt") == 0)
			{
				if (angle > TILT_UPPER_ANGLE_LIMIT)
				{
					angle = TILT_UPPER_ANGLE_LIMIT;
				}
				else if (angle < TILT_LOWER_ANGLE_LIMIT)
				{
					angle = TILT_LOWER_ANGLE_LIMIT;
				}
				ptz.SetTilting(angle);
			}
			else
			{
				std::cout << -999 << std::endl;
				std::cout << -999;
				return -1;
			}
		}
	}
	std::cout << ptz.GetPanning() << std::endl;
	std::cout << ptz.GetTilting();
	return 999;
}