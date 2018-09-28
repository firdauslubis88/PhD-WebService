#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include "opencv2/opencv.hpp"
#include "ofQuaternion.h"
#include <omp.h>
#include "ceres/ceres.h"
#include "glog/logging.h"

const int THETA_V_WIDTH = 3840/2;
const int THETA_V_HEIGHT = 1920/2;
const int PTZ_WIDTH = 1920/2;
const int PTZ_HEIGHT = 1080/2;
static const double APAP_GAMMA = 0.0015;
static const double APAP_SIGMA = 8.5;
const int GRID_SIZE = 40;

typedef float FLOAT_TYPE;
typedef cv::Point_<FLOAT_TYPE> Point2;

template <typename T>
cv::Point_<T> applyTransform3x3(T x, T y, const cv::Mat & matT) {
	double denom = 1. / (matT.at<double>(2, 0) * x + matT.at<double>(2, 1) * y + matT.at<double>(2, 2));
	return cv::Point_<T>((matT.at<double>(0, 0) * x + matT.at<double>(0, 1) * y + matT.at<double>(0, 2)) * denom,
		(matT.at<double>(1, 0) * x + matT.at<double>(1, 1) * y + matT.at<double>(1, 2)) * denom);
}

template cv::Point_< float> applyTransform3x3< float>(float x, float y, const cv::Mat & matT);
template cv::Point_<double> applyTransform3x3<double>(double x, double y, const cv::Mat & matT);

//--------------------------------------------------
float ofRadToDeg(float radians) {
	return radians * RAD_TO_DEG;
}

//--------------------------------------------------
float ofDegToRad(float degrees) {
	return degrees * DEG_TO_RAD;
}

static cv::Mat getNormalize2DPts(const std::vector<cv::Point2f> & pts, std::vector<cv::Point2f> & newpts) {
	cv::Mat pts_ref(pts), npts;
	cv::Scalar mean_p = mean(pts_ref);
	npts = pts_ref - mean_p;
	cv::Mat dist = npts.mul(npts);
	dist = dist.reshape(1);
	sqrt(dist.col(0) + dist.col(1), dist);
	double scale = sqrt(2) / mean(dist).val[0];

	cv::Mat result(3, 3, CV_64FC1);
	result.at<double>(0, 0) = scale;
	result.at<double>(0, 1) = 0;
	result.at<double>(0, 2) = -scale * (double)mean_p.val[0];

	result.at<double>(1, 0) = 0;
	result.at<double>(1, 1) = scale;
	result.at<double>(1, 2) = -scale * (double)mean_p.val[1];

	result.at<double>(2, 0) = 0;
	result.at<double>(2, 1) = 0;
	result.at<double>(2, 2) = 1;

#ifndef NDEBUG
	if (newpts.empty() == false) {
		newpts.clear();
		printf("F(getNormalize2DPts) newpts is not empty");
	}
#endif
	newpts.reserve(pts.size());
	for (int i = 0; i < pts.size(); ++i) {
		newpts.emplace_back(pts[i].x * result.at<double>(0, 0) + result.at<double>(0, 2),
			pts[i].y * result.at<double>(1, 1) + result.at<double>(1, 2));
	}

	return result;
}

static cv::Mat getConditionerFromPts(const std::vector<cv::Point2f> & pts) {
	cv::Mat pts_ref(pts);
	cv::Scalar mean_pts, std_pts;
	meanStdDev(pts_ref, mean_pts, std_pts);

	std_pts = (std_pts.mul(std_pts) * pts_ref.rows / (double)(pts_ref.rows - 1));
	sqrt(std_pts, std_pts);

	std_pts.val[0] = std_pts.val[0] + (std_pts.val[0] == 0);
	std_pts.val[1] = std_pts.val[1] + (std_pts.val[1] == 0);

	cv::Mat result(3, 3, CV_64FC1);
	result.at<double>(0, 0) = sqrt(2) / (double)std_pts.val[0];
	result.at<double>(0, 1) = 0;
	result.at<double>(0, 2) = -(sqrt(2) / (double)std_pts.val[0]) * (double)mean_pts.val[0];

	result.at<double>(1, 0) = 0;
	result.at<double>(1, 1) = sqrt(2) / (double)std_pts.val[1];
	result.at<double>(1, 2) = -(sqrt(2) / (double)std_pts.val[1]) * (double)mean_pts.val[1];

	result.at<double>(2, 0) = 0;
	result.at<double>(2, 1) = 0;
	result.at<double>(2, 2) = 1;

	return result;
}

void getVertices(std::vector<Point2>& vertices, int _cols, int _rows) {
	int nw = _cols / GRID_SIZE + (_cols % GRID_SIZE != 0);
	int nh = _rows / GRID_SIZE + (_rows % GRID_SIZE != 0);
	double lw = _cols / (double)nw;
	double lh = _rows / (double)nh;

	const int memory = (nh + 1) * (nw + 1);
	vertices.reserve(memory);
	for (int h = 0; h <= nh; ++h) {
		for (int w = 0; w <= nw; ++w) {
			vertices.emplace_back(static_cast<float>(w * lw), static_cast<float>(h * lh));
		}
	}
}

cv::Mat getImageOfFeaturePairs(const cv::Mat & img1,
	const cv::Mat & img2,
	const std::vector<Point2> & f1,
	const std::vector<Point2> & f2) {
	assert(f1.size() == f2.size());
	assert(img1.type() == img2.type());

	const int CIRCLE_RADIUS = 5;
	const int CIRCLE_THICKNESS = 1;
	const int LINE_THICKNESS = 1;
	const int RGB_8U_RANGE = 256;

	cv::Mat result = cv::Mat::zeros(std::max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
	cv::Mat left(result, cv::Rect(0, 0, img1.cols, img1.rows));
	cv::Mat right(result, cv::Rect(img1.cols, 0, img2.cols, img2.rows));

	cv::Mat img1_8UC3, img2_8UC3;

	if (img1.type() == CV_8UC3) {
		img1_8UC3 = img1;
		img2_8UC3 = img2;
	}
	else {
		img1.convertTo(img1_8UC3, CV_8UC3);
		img2.convertTo(img2_8UC3, CV_8UC3);
	}
	img1_8UC3.copyTo(left);
	img2_8UC3.copyTo(right);

	for (int i = 0; i < f1.size(); ++i) {
		cv::Scalar color(rand() % RGB_8U_RANGE, rand() % RGB_8U_RANGE, rand() % RGB_8U_RANGE);
		circle(result, f1[i], CIRCLE_RADIUS, color, CIRCLE_THICKNESS, cv::LINE_AA);
		line(result, f1[i], f2[i] + Point2(static_cast<float>(img1.cols), 0.), color, LINE_THICKNESS, cv::LINE_AA);
		circle(result, f2[i] + Point2(static_cast<float>(img1.cols), 0.), CIRCLE_RADIUS, color, CIRCLE_THICKNESS, cv::LINE_AA);
	}
	return result;
}

struct CostFunctor {
	CostFunctor(double* x) {
		x_ = new double[9];
		for (size_t i = 0; i < 9; i++)x_[i] = x[i];
	}

	~CostFunctor() {
		delete x_;
	}

	template <typename T> bool operator()(const T* const h, T* residual) const {
		residual[0] = T(x_[0]) * h[0] + T(x_[1]) * h[1] + T(x_[2]) * h[2]
			+ T(x_[3]) * h[3] + T(x_[4]) * h[4] + T(x_[5]) * h[5]
			+ T(x_[6]) * h[6] + T(x_[7]) * h[7] + T(x_[8]) * h[8];
		return true;
	}

private:
	double* x_;
};

void apap(const std::vector<Point2> & _p_src,
	const std::vector<Point2> & _p_dst,
	const std::vector<Point2> & _src,
	std::vector<Point2> & _dst,
	std::vector<cv::Mat> & _homographies) {
	std::vector<Point2> nf1, nf2, cf1, cf2;
	cv::Mat N1, N2, C1, C2;
	N1 = getNormalize2DPts(_p_src, nf1);
	N2 = getNormalize2DPts(_p_dst, nf2);
	C1 = getConditionerFromPts(nf1);
	C2 = getConditionerFromPts(nf2);
	cf1.reserve(nf1.size());
	cf2.reserve(nf2.size());
	for (int i = 0; i < nf1.size(); ++i) {
		cf1.emplace_back(static_cast<float>(nf1[i].x * C1.at<double>(0, 0) + C1.at<double>(0, 2)),
			static_cast<float>(nf1[i].y * C1.at<double>(1, 1) + C1.at<double>(1, 2)));

		cf2.emplace_back(static_cast<float>(nf2[i].x * C2.at<double>(0, 0) + C2.at<double>(0, 2)),
			static_cast<float>(nf2[i].y * C2.at<double>(1, 1) + C2.at<double>(1, 2)));
	}
	double sigma_inv_2 = 1. / (APAP_SIGMA * APAP_SIGMA), gamma = APAP_GAMMA;

//#ifndef NDEBUG
//	if (_dst.empty() == false) {
//		_dst.clear();
//		printf("F(apap_project) dst is not empty");
//	}
//	if (_homographies.empty() == false) {
//		_homographies.clear();
//		printf("F(apap_project) homographies is not empty");
//	}
//#endif
	_dst.reserve(_src.size());
	_homographies.reserve(_src.size());
	for (int i = 0; i < _src.size(); ++i) {
		ceres::Problem problem;
		double h[9] = { 0,0,0,0,0,0,0,0,1 };
		for (int j = 0; j < _p_src.size(); ++j) {
			Point2 d = _src[i] - _p_src[j];
			double www = MAX(gamma, exp(-sqrt(d.x * d.x + d.y * d.y) * sigma_inv_2));
			double x[9], y[9];

			x[0] = www * cf1[j].x;
			x[1] = www * cf1[j].y;
			x[2] = www * 1;
			x[3] = 0;
			x[4] = 0;
			x[5] = 0;
			x[6] = www * -cf2[j].x * cf1[j].x;
			x[7] = www * -cf2[j].x * cf1[j].y;
			x[8] = www * -cf2[j].x;

			y[0] = 0;
			y[1] = 0;
			y[2] = 0;
			y[3] = www * cf1[j].x;
			y[4] = www * cf1[j].y;
			y[5] = www * 1;
			y[6] = www * -cf2[j].y * cf1[j].x;
			y[7] = www * -cf2[j].y * cf1[j].y;
			y[8] = www * -cf2[j].y;

			// Set up the only cost function (also known as residual). This uses
			// auto-differentiation to obtain the derivative (jacobian).
			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<CostFunctor, 1, 9>(new CostFunctor(x));
			ceres::CostFunction* cost_function_y =
				new ceres::AutoDiffCostFunction<CostFunctor, 1, 9>(new CostFunctor(y));
			problem.AddResidualBlock(cost_function, NULL, h);
			problem.AddResidualBlock(cost_function_y, NULL, h);
		}
			// Run the solver!
		ceres::Solver::Options options;
		options.minimizer_progress_to_stdout = false;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		cv::Mat H(3, 3, CV_64FC1);
		for (int j = 0; j < 9; ++j) {
			H.at<double>(j / 3, j % 3) = h[j];
		}
		H = C2.inv() * H * C1;
		H = N2.inv() * H * N1;

		_dst.emplace_back(applyTransform3x3(_src[i].x, _src[i].y, H));
		_homographies.emplace_back(H);
	}
}

static ofVec2f planarToEquirectangular(const ofVec2f& planarCoord, const float& vFOV, const float& aspectRatio,
	const float& imagePlaneHeight, const ofQuaternion& quatInverse, const bool& inverse, const float& equiHeight)
{
	float vFOV_ = vFOV * PI / 180; float height = imagePlaneHeight; float width = height * aspectRatio;
	float scale = equiHeight / 180;
	float equiWidth = equiHeight * 2;
	float f = imagePlaneHeight / (2 * tanf(vFOV_ / 2));
	float ic = planarCoord.x - width / 2, jc = planarCoord.y - height / 2;

//	ofVec3f euler = currentQuat.getEuler();
//	ofQuaternion quatX;
//	quatX.makeRotate(-euler.x, 1, 0, 0);
//	ofQuaternion quatY;
//	quatY.makeRotate(euler.y, 0, 1, 0);
//	ofQuaternion quatZ;
//	quatZ.makeRotate(euler.z, 0, 0, 1);

	ofVec3f rotateResult;
	float globalPhi, globalTheta;
//	ofQuaternion quatInverse = quatY * quatZ * quatX;
//	quatInverse = quatInverse.inverse();
//	ofQuaternion quatInverse = currentQuat.inverse();
	rotateResult = quatInverse*ofVec3f(ic, jc, f);

	globalPhi = atanf(rotateResult.y / sqrtf(rotateResult.x*rotateResult.x + rotateResult.z * rotateResult.z));
	if (rotateResult.z > 0)
	{
		globalTheta = atanf(rotateResult.x / rotateResult.z);
	}
	else
	{
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
//	return ofVec2f(globalPhi, globalTheta);
//	return ofVec2f(ic, jc);
}

static void updateOmniMat(const cv::Mat& mat360, cv::Mat& mat, const int& viewportWidth, const int& viewportHeight, const float& vFOV, const float& aspectRatio, 
	const float& zAngle, const ofQuaternion& orientationQuat, const int& vHeight, const int& realHeight)
{
	std::pair<cv::Mat, cv::Mat> tmpMap;
	tmpMap.first = cv::Mat(viewportHeight, viewportWidth, CV_32FC1);
	tmpMap.second = cv::Mat(viewportHeight, viewportWidth, CV_32FC1);
//	const ofQuaternion quatInverse = orientationQuat.inverse();
	ofQuaternion quatZ;
	quatZ.makeRotate(-zAngle, 0, 0, 1);
	ofQuaternion tmpQuat = orientationQuat * quatZ;
	const ofQuaternion quatInverse = tmpQuat.inverse();
#pragma omp parallel for
	for (int j = 0; j < viewportHeight; j++)
	{
		for (int i = 0; i < viewportWidth; i++)
		{
			ofVec2f planarCoord(i, j);
//			ofQuaternion tmpQuat = orientationQuat;
			ofVec2f equiCoord = planarToEquirectangular(planarCoord, vFOV, aspectRatio,
				vHeight, quatInverse, true, realHeight);

			tmpMap.first.at<float>(j, i) = equiCoord.x;
			tmpMap.second.at<float>(j, i) = equiCoord.y;
		}
	}
//	this->updateMat360();
//	cv::cuda::GpuMat d_src, d_dst, d_xmap, d_ymap;
//	d_src.upload(mat360);
//	d_xmap.upload(tmpMap.first);																												//uncomment these later!!!											//real mapping done. These are only for debugging purpose
//	d_ymap.upload(tmpMap.second);																											//uncomment these later!!!											//real mapping done. These are only for debugging purpose
//
//	cv::cuda::remap(d_src, d_dst, d_xmap, d_ymap, CV_INTER_LINEAR, cv::BORDER_CONSTANT);
//	d_dst.download(mat);

	double start_time = omp_get_wtime();
	cv::remap(mat360, mat, tmpMap.first, tmpMap.second, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	double end_time = omp_get_wtime();
//	std::cout << "remap time:\t" << end_time - start_time << std::endl;
}

static float hFOVToVFOV(float hFOV, float aspectRatio)
{
	return 2 * atanf(tanf(hFOV * PI / 180 / 2) / aspectRatio) * 180 / PI;
}

int main(int argc, char** argv)
{
	int input = 0;
	if (argc > 1)
	{
		sscanf(argv[1], "%d", &input);
	}
	std::cout << input;

//	cv::namedWindow("test");
	cv::Mat theta = cv::imread("E:\\repos\\PhD-WebService\\thetaInput-half.png");
//	cv::imshow("test",theta);
//	cv::waitKey(0);
	cv::Mat ptz = cv::imread("E:\\repos\\PhD-WebService\\ptzInput-half.png");
//	cv::imshow("test", ptz);
//	cv::waitKey(0);
//	cv::destroyAllWindows();
	cv::Mat theta_planar;
	const float aspectRatio = 16. / 9.;
	const int realWidth = THETA_V_WIDTH, realHeight = THETA_V_HEIGHT, viewportWidth = PTZ_WIDTH, viewportHeight = PTZ_HEIGHT, vHeight = PTZ_HEIGHT;
	const float thetahFOV = 82.1f, thetavFOV = hFOVToVFOV(thetahFOV, aspectRatio);
	const float zAngle = -90;
	const ofQuaternion orientationQuat = ofQuaternion(0, 0, 0, 1);

	double start_time = omp_get_wtime();
	updateOmniMat(theta, theta_planar, viewportWidth, viewportHeight, thetavFOV, aspectRatio,
		zAngle, orientationQuat, vHeight, realHeight);
	double end_time = omp_get_wtime();
	std::cout << "preparation time:\t" << end_time - start_time << std::endl;

	cv::Mat descriptors1;
	cv::Mat descriptors2;
	std::vector<cv::KeyPoint> kPoint1;
	std::vector<cv::KeyPoint> kPoint2;

	cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
	start_time = omp_get_wtime();
	orb->detectAndCompute(theta_planar, cv::Mat(), kPoint2, descriptors2);
	orb->detectAndCompute(ptz, cv::Mat(), kPoint1, descriptors1);
	end_time = omp_get_wtime();
	std::cout << "detection time:\t" << end_time - start_time << std::endl;
	std::cout << "Keypoint1: " << kPoint1.size() << ", Keypoint2: " << kPoint2.size() << std::endl;
	std::cout << "Descriptor1: " << descriptors1.size() << ", Descriptors2: " << descriptors2.size() << std::endl;

	cv::FlannBasedMatcher matcher;
	std::vector<std::vector<cv::DMatch>> kmatches;
	std::vector<cv::DMatch> good_matches;
	std::vector<cv::Point2f> ptz_features, theta_features;
	float comparisonThreshold = 0.7f;
	start_time = omp_get_wtime();
	if (descriptors1.type() != CV_32F) {
		descriptors1.convertTo(descriptors1, CV_32F);
	}

	if (descriptors2.type() != CV_32F) {
		descriptors2.convertTo(descriptors2, CV_32F);
	}
	matcher.knnMatch(descriptors1, descriptors2, kmatches, 2);
	std::sort(kmatches.begin(), kmatches.end());
	int lenghtKMatches = static_cast<int>(kmatches.size());

	for (int i = 0; i < lenghtKMatches; i++)
	{
		double dist1 = kmatches[i][0].distance;
		double dist2 = kmatches[i][1].distance;
		double comp = dist1 / dist2;

		if (comp < comparisonThreshold)
		{
			good_matches.push_back(kmatches[i][0]);
			ptz_features.push_back(kPoint1[kmatches[i][0].queryIdx].pt);
			theta_features.push_back(kPoint2[kmatches[i][0].trainIdx].pt);
		}
	}

	std::vector<char> final_mask(good_matches.size(), 0);
	cv::Mat h = cv::findHomography(ptz_features, theta_features, cv::RANSAC, 3, final_mask);
	std::vector<cv::Point2f> ptz_features_filtered, theta_features_filtered;
	for (int i = 0; i < final_mask.size(); ++i) {
		if (final_mask[i]) {
			ptz_features_filtered.push_back(ptz_features[i]);
			theta_features_filtered.push_back(theta_features[i]);
		}
	}
	end_time = omp_get_wtime();
//	cv::Mat img_matches;
//	cv::drawMatches(ptz, kPoint1, theta_planar, kPoint2,
//		good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	std::cout << "matching time:\t" << end_time - start_time << std::endl;
	std::cout << "matches: " << ptz_features_filtered.size() << std::endl;

	std::vector<Point2> origin_vertices, stitched_vertices;
	std::vector<cv::Mat> homographies;
	getVertices(origin_vertices, ptz.cols, ptz.rows);
	apap(ptz_features_filtered, theta_features_filtered, origin_vertices, stitched_vertices,
		homographies);
	std::cout << "origin vertices size:\t" << origin_vertices.size() << std::endl;
	std::cout << "stitched vertices size:\t" << stitched_vertices.size() << std::endl;
	for (size_t i = 0; i < stitched_vertices.size(); i++)
	{
		std::cout << origin_vertices[i] << "\t" << stitched_vertices[i] << std::endl;
	}

	cv::Mat img_matches = getImageOfFeaturePairs(ptz, theta_planar, ptz_features_filtered, theta_features_filtered);
	cv::namedWindow("test");
	cv::imshow("test", img_matches);
	cv::waitKey(0);
	cv::destroyAllWindows();
//	std::system("pause");

}
#undef _CRT_SECURE_NO_WARNINGS