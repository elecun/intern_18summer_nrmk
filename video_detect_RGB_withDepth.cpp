// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "opencv2/xfeatures2d.hpp"
#include <algorithm>
#include <iostream>
#include <string>
#include <windows.h>			// To use Slepp() function. No need if useless
#include "C:\opencv\librealsense-master\librealsense-master\examples\C\example.h"

#pragma comment(lib, "realsense2.lib")
#pragma comment(lib, "realsense-file.lib")
#pragma comment(lib, "libcpmt.lib")

#define MIN_DEPTH		0.40			// Defines the minimum depth
#define	MAX_DEPTH		0.50			  // Defines the maximum depth (hegith from camera to floor)
#define MIN_AREA		100			  // Defines the minum area of components
#define WIDTH           848               // Defines the number of columns for each frame                         
#define HEIGHT          480               // Defines the number of lines for each frame                           
#define FPS             6                // Defines the rate of frames per second     
#define STREAM_COLOR    RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT_COLOR    RS2_FORMAT_RGB8   // rs2_format is identifies how binary data is encoded within a frame   //
#define STREAM_DEPTH    RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           
#define FORMAT_DEPTH    RS2_FORMAT_Z16    // rs2_format is identifies how binary data is encoded within a frame   
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type 
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

float get_depth_unit_value(const rs2_device* const dev)
{
	rs2_error* e = 0;
	rs2_sensor_list* sensor_list = rs2_query_sensors(dev, &e);
	check_error(e);

	int num_of_sensors = rs2_get_sensors_count(sensor_list, &e);
	check_error(e);

	float depth_scale = 0;
	int is_depth_sensor_found = 0;
	int i;
	for (i = 0; i < num_of_sensors; ++i)
	{
		rs2_sensor* sensor = rs2_create_sensor(sensor_list, i, &e);
		check_error(e);

		// Check if the given sensor can be extended to depth sensor interface
		is_depth_sensor_found = rs2_is_sensor_extendable_to(sensor, RS2_EXTENSION_DEPTH_SENSOR, &e);
		check_error(e);

		if (1 == is_depth_sensor_found)
		{
			depth_scale = rs2_get_option((const rs2_options*)sensor, RS2_OPTION_DEPTH_UNITS, &e);
			check_error(e);
			rs2_delete_sensor(sensor);
			break;
		}
		rs2_delete_sensor(sensor);
	}
	rs2_delete_sensor_list(sensor_list);

	if (0 == is_depth_sensor_found)
	{
		printf("Depth sensor not found!\n");
		exit(EXIT_FAILURE);
	}

	return depth_scale;
}

int main(int argc, char * argv[]) try
{
	rs2_error* e = 0;

	// Create a context object. This object owns the handles to all connected realsense devices.
	rs2_context* ctx_color = rs2_create_context(RS2_API_VERSION, &e);	check_error(e);
	rs2_device_list* device_list_color = rs2_query_devices(ctx_color, &e);	check_error(e);

	int dev_count_color = rs2_get_device_count(device_list_color, &e);		check_error(e);
	printf("There are %d connected RealSense devices.\n", dev_count_color - 1);
	if (0 == dev_count_color) { return EXIT_FAILURE; }
	rs2_device* dev_color = rs2_create_device(device_list_color, 0, &e);	 check_error(e);
	print_device_info(dev_color);
	rs2_pipeline* pipeline_color = rs2_create_pipeline(ctx_color, &e);		check_error(e);
	rs2_config* config_color = rs2_create_config(&e);					check_error(e);
	rs2_config_enable_stream(config_color, STREAM_COLOR, STREAM_INDEX, WIDTH, HEIGHT, FORMAT_COLOR, FPS, &e);	check_error(e);
	rs2_pipeline_profile* pipe_color = rs2_pipeline_start_with_config(pipeline_color, config_color, &e);


	// Second
	rs2_context* ctx_depth = rs2_create_context(RS2_API_VERSION, &e);	check_error(e);
	rs2_device_list* device_list_depth = rs2_query_devices(ctx_depth, &e);	check_error(e);

	int dev_count_depth = rs2_get_device_count(device_list_depth, &e);		check_error(e);
	printf("There are %d connected RealSense devices.\n", dev_count_depth - 1);
	if (0 == dev_count_depth) { return EXIT_FAILURE; }
	rs2_device* dev_depth = rs2_create_device(device_list_depth, 0, &e);	 check_error(e);
	print_device_info(dev_depth);
	rs2_pipeline* pipeline_depth = rs2_create_pipeline(ctx_depth, &e);		check_error(e);
	rs2_config* config_depth = rs2_create_config(&e);					check_error(e);
	rs2_config_enable_stream(config_depth, STREAM_DEPTH, STREAM_INDEX, WIDTH, HEIGHT, FORMAT_DEPTH, FPS, &e);	check_error(e);
	rs2_pipeline_profile* pipe_depth = rs2_pipeline_start_with_config(pipeline_depth, config_depth, &e);


	if (e)
	{
		printf("The connected device doesn't support depth && color streaming!\n");
		exit(EXIT_FAILURE);
	}

	uint16_t one_meter = (uint16_t)(1.0f / get_depth_unit_value(dev_depth));
	while (waitKey(1) < 0)
	{
		rs2_frame* frames_color = rs2_pipeline_wait_for_frames(pipeline_color, 5000, &e);
		//rs2_frame* frames_depth = rs2_pipeline_wait_for_frames(pipeline_depth, 5000, &e);

		int num_of_frames = rs2_embedded_frames_count(frames_color, &e);	check_error(e);
		FlannBasedMatcher matcher;
		for (int i = 0; i < num_of_frames; ++i)
		{
			rs2_frame* frame_color = rs2_extract_frame(frames_color, i, &e);	check_error(e);
			const uint16_t* color_frame_data = (const uint16_t*)(rs2_get_frame_data(frame_color, &e));	check_error(e);

			// STEP 1 : Read Image

			Mat Scene(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame_data, Mat::AUTO_STEP);
			//imshow("image", Scene);

			Mat Marker = imread("C:\\Users\\JaehyunKim\\Desktop\\Marker.jpg", IMREAD_COLOR);
			if (Marker.empty() || Scene.empty()) {
				cout << "Could not open or find the image" << endl;
				return -1;
			}
			Mat gMarker(Marker.size(), CV_8UC1);
			Mat gScene(Scene.size(), CV_8UC1);
			cvtColor(Marker, gMarker, COLOR_BGR2GRAY);
			cvtColor(Scene, gScene, COLOR_BGR2GRAY);

			imshow("input", gMarker);
			imshow("Gray Scene", gScene);


			// STEP 2 : Detect Feature Points

			int minHessian = 400;
			Mat key_gMarker, key_gScene, descriptors_1, descriptors_2;

			Ptr<SURF> detector = SURF::create();
			detector->setHessianThreshold(minHessian);
			std::vector<KeyPoint> keypoints_1, keypoints_2;

			detector->detectAndCompute(gMarker, Mat(), keypoints_1, descriptors_1);
			detector->detectAndCompute(gScene, Mat(), keypoints_2, descriptors_2);


			// STEP 3 : Extract Feature Descriptors


			std::vector< DMatch > matches;

			matcher.match(descriptors_1, descriptors_2, matches);		// THIS LINE produces ASSERTION ERROR / RUNTIME ERROR
			//Sleep(1000);

			/*
			double max_dist = 0; double min_dist = 100;

			for (int j = 0; j < descriptors_1.rows; j++) {
			double dist = matches[j].distance;
			if (dist < min_dist)
			min_dist = dist;
			if (dist > max_dist)
			max_dist = dist;
			}
			printf("-- Max dist : %f \n", max_dist);
			printf("-- Min dist : %f \n", min_dist);


			// STEP 4 : Find Putative Match Points and Match two images

			Mat match;

			std::vector< DMatch > good_matches;
			for (int k = 0; k < descriptors_1.rows; k++) {
			if (matches[k].distance <= max(2 * min_dist, 0.02)) {
			good_matches.push_back(matches[k]);
			}
			}

			drawMatches(gMarker, keypoints_1, gScene, keypoints_2, good_matches, match, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			imshow("Good Matches", match);

			for (int i = 0; i < (int)good_matches.size(); i++) {
			printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
			}

			// STEP 5 : Locating Marker in the Scene

			vector<Point2f> obj;
			vector<Point2f> scene;

			for (int i = 0; i < good_matches.size(); i++) {
			obj.push_back(keypoints_1[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_2[good_matches[i].trainIdx].pt);
			}

			Mat H = findHomography(obj, scene, RANSAC);

			vector<Point2f> objCorners(4);
			objCorners[0] = cvPoint(0, 0);
			objCorners[1] = cvPoint(gMarker.cols, 0);
			objCorners[2] = cvPoint(gMarker.cols, gMarker.rows);
			objCorners[3] = cvPoint(0, gMarker.rows);
			vector<Point2f> sceneCorners(4);

			perspectiveTransform(objCorners, sceneCorners, H);

			
			//	VISUALIZE Detected Marker Location

			line(match, sceneCorners[0] + Point2f(gMarker.cols, 0), sceneCorners[1] + Point2f(gMarker.cols, 0), Scalar(0, 255, 0), 4);
			line(match, sceneCorners[1] + Point2f(gMarker.cols, 0), sceneCorners[2] + Point2f(gMarker.cols, 0), Scalar(0, 255, 0), 4);
			line(match, sceneCorners[2] + Point2f(gMarker.cols, 0), sceneCorners[3] + Point2f(gMarker.cols, 0), Scalar(0, 255, 0), 4);
			line(match, sceneCorners[3] + Point2f(gMarker.cols, 0), sceneCorners[0] + Point2f(gMarker.cols, 0), Scalar(0, 255, 0), 4);

			imshow("DETECTION COMPLETED", match);
			*/
			rs2_release_frame(frame_color);
		}
		rs2_release_frame(frames_color);
		//rs2_release_frame(frames_depth);

	}
	rs2_pipeline_stop(pipeline_color, &e);
	check_error(e);
	rs2_delete_pipeline_profile(pipe_color);
	rs2_delete_config(config_color);
	rs2_delete_pipeline(pipeline_color);
	rs2_delete_device(dev_color);
	rs2_delete_device_list(device_list_color);
	rs2_delete_context(ctx_color);


	rs2_pipeline_stop(pipeline_depth, &e);
	check_error(e);
	rs2_delete_pipeline_profile(pipe_depth);
	rs2_delete_config(config_depth);
	rs2_delete_pipeline(pipeline_depth);
	rs2_delete_device(dev_depth);
	rs2_delete_device_list(device_list_depth);
	rs2_delete_context(ctx_depth);

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}



