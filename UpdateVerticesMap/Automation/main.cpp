#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include "opencv2/opencv.hpp"

#include "ofQuaternion.h"

const int THETA_V_WIDTH = 3840;
const int THETA_V_HEIGHT = 1920;
const int PTZ_WIDTH = 1920;
const int PTZ_HEIGHT = 1080;

//--------------------------------------------------
float ofRadToDeg(float radians) {
	return radians * RAD_TO_DEG;
}

//--------------------------------------------------
float ofDegToRad(float degrees) {
	return degrees * DEG_TO_RAD;
}

static ofVec2f planarToEquirectangular(ofVec2f planarCoord, float vFOV, float aspectRatio,
	float imagePlaneHeight, ofQuaternion currentQuat, bool inverse, float equiHeight)
{
	vFOV = vFOV * PI / 180; float height = imagePlaneHeight; float width = height * aspectRatio;
	float scale = equiHeight / 180;
	float equiWidth = equiHeight * 2;
	float f = imagePlaneHeight / (2 * tanf(vFOV / 2));
	float ic = planarCoord.x - width / 2, jc = planarCoord.y - height / 2;

	ofVec3f euler = currentQuat.getEuler();
	ofQuaternion quatX;
	quatX.makeRotate(-euler.x, 1, 0, 0);
	ofQuaternion quatY;
	quatY.makeRotate(euler.y, 0, 1, 0);
	ofQuaternion quatZ;
	quatZ.makeRotate(euler.z, 0, 0, 1);

	ofVec3f rotateResult;
	float globalPhi, globalTheta;
	if (!inverse)
	{
		rotateResult = quatY * quatZ * quatX * ofVec3f(ic, jc, f);
	}
	else
	{
		ofQuaternion quatInverse = quatY * quatZ * quatX;
		quatInverse = quatInverse.inverse();
		rotateResult = quatInverse * ofVec3f(ic, jc, f);
	}
	if (rotateResult.z > 0)
	{
		globalPhi = atanf(rotateResult.y / sqrtf(rotateResult.x*rotateResult.x + rotateResult.z * rotateResult.z));
		globalTheta = atanf(rotateResult.x / rotateResult.z);
	}
	else
	{
		globalPhi = atanf(rotateResult.y / sqrtf(rotateResult.x*rotateResult.x + rotateResult.z * rotateResult.z));
		globalTheta = (atanf(rotateResult.x / rotateResult.z) - PI);
		if (globalTheta < -PI)
		{
			globalTheta += 2 * PI;
		}
	}
	globalPhi = globalPhi * 180 / PI; globalTheta = globalTheta * 180 / PI;
	float x = scale * globalTheta + equiWidth / 2;
	float y = scale * globalPhi + equiHeight / 2;

	return ofVec2f(x, y);
}

static void updateOmniMat(cv::Mat& mat360, cv::Mat mat, int viewportWidth, int viewportHeight, float vFOV, float aspectRatio, 
	float zAngle, ofQuaternion orientationQuat, int vHeight, int realHeight)
{
	std::pair<cv::Mat, cv::Mat> tmpMap;
	tmpMap.first = cv::Mat(viewportHeight, viewportWidth, CV_32FC1);
	tmpMap.second = cv::Mat(viewportHeight, viewportWidth, CV_32FC1);

	for (int j = 0; j < viewportHeight; j++)
	{
		for (int i = 0; i < viewportWidth; i++)
		{
			ofVec2f planarCoord(i, j);
			ofQuaternion quatZ;
			quatZ.makeRotate(-zAngle, 0, 0, 1);
			ofQuaternion tmpQuat = orientationQuat;
			tmpQuat = tmpQuat * quatZ;
			ofVec2f equiCoord = planarToEquirectangular(planarCoord, vFOV, aspectRatio,
				vHeight, tmpQuat, true, realHeight);
			float x = equiCoord.x;
			float y = equiCoord.y;

			tmpMap.first.at<float>(j, i) = x;
			tmpMap.second.at<float>(j, i) = y;
		}
	}

//	this->updateMat360();
	cv::remap(mat360, mat, tmpMap.first, tmpMap.second, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

int main(int argc, char** argv)
{
	int input = 0;
	if (argc > 1)
	{
		sscanf(argv[1], "%d", &input);
	}
	std::cout << input;

	cv::namedWindow("test");
	cv::Mat theta = cv::imread("D:\\repos\\PhD-WebService\\thetaInput.png");
	cv::imshow("test",theta);
	cv::waitKey(0);
	cv::Mat ptz = cv::imread("D:\\repos\\PhD-WebService\\ptzInput.png");
	cv::imshow("test", ptz);
	cv::waitKey(0);
	cv::destroyAllWindows();

}
#undef _CRT_SECURE_NO_WARNINGS