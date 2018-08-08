// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <algorithm>
#include "C:\opencv\librealsense-master\librealsense-master\examples\C\example.h"

#define MIN_DEPTH		0.47				// Defines the minimum depth
#define	MAX_DEPTH		0.50			  // Defines the maximum depth (hegith from camera to floor)
#define MIN_AREA		1000			  // Defines the minum area of components
#define WIDTH           848               // Defines the number of columns for each frame                         
#define HEIGHT          480               // Defines the number of lines for each frame                           
#define FPS             6                // Defines the rate of frames per second                                
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           
#define FORMAT          RS2_FORMAT_Z16    // rs2_format is identifies how binary data is encoded within a frame   
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type 
using namespace cv;

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
	rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);	check_error(e);
	rs2_device_list* device_list = rs2_query_devices(ctx, &e);	check_error(e);

	int dev_count = rs2_get_device_count(device_list, &e);		check_error(e);
	printf("There are %d connected RealSense devices.\n", dev_count-1);
	if (0 == dev_count)	{return EXIT_FAILURE;}
	rs2_device* dev = rs2_create_device(device_list, 0, &e);	 check_error(e);
	print_device_info(dev);
	rs2_pipeline* pipeline = rs2_create_pipeline(ctx, &e);		check_error(e);
	rs2_config* config = rs2_create_config(&e);					check_error(e);
	rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);	check_error(e);
	rs2_pipeline_profile* pipe = rs2_pipeline_start_with_config(pipeline, config, &e);
	if (e)
	{
		printf("The connected device doesn't support depth streaming!\n");
		exit(EXIT_FAILURE);
	}
	double th = MAX_DEPTH;
	uint16_t one_meter = (uint16_t)(1.0f / get_depth_unit_value(dev));
	while (waitKey(1) < 0)
	{
		rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, 5000, &e);
		int num_of_frames = rs2_embedded_frames_count(frames, &e);	check_error(e);
		
		int i;
		for (i = 0; i < num_of_frames; ++i)
		{
			rs2_frame* frame = rs2_extract_frame(frames, i, &e);	check_error(e);
			/*if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
			{	rs2_release_frame(frame);	continue;	}
			*/
			//	VIDEO PIPELINING EVIRONMENT SETTING DONE
			/*
				OBTAIN RAW DEPTH DATA FROM CONNECTED INTEL REALSENSE DEVICE
				USER VARIABLE	:	WIDTH, HEIGHT
			*/

			const uint16_t* depth_frame_data = (const uint16_t*)(rs2_get_frame_data(frame, &e));	check_error(e);
			Mat image(Size(WIDTH, HEIGHT), CV_16UC1, (void *)depth_frame_data, Mat::AUTO_STEP);

			/*
				THRESHOLDS IMAGE WITH OPENCV FUNCTION
				USER VARIABLE	:	MAX_DEPTH, MIN_DEPTH
			*/

			Mat image_thres1,image_thres;
			threshold(image, image_thres1, (th)*one_meter, 65535, THRESH_TOZERO_INV);
			threshold(image_thres1, image_thres, (th-0.005)*one_meter, 65535, THRESH_BINARY);
			image_thres.convertTo(image_thres, CV_8U, 1 / 256.0);
			th += 0.001;
			if (th >= MAX_DEPTH)
			{
				th = MIN_DEPTH;
			}
			/*
				CLUSTERS THRESHOLDED IMAGE WITH OPENCV FUNCTION
				PRODUCES MATRIX OF LABELED AREA INFOS OF EACH COMPONENTS 
			*/
			Mat image_label, stats, cens;
			int x, y, obj_count = 0;
			int obj[21] = { 0 };
			int components = components = connectedComponentsWithStats(image_thres, image_label, stats, cens);
			
			std::vector<Vec3b> colors(components + 1);
			colors[0] = Vec3b(0, 0, 0);

			/*
				IGNORING NOISES BY FILTERING SMALL AREA COMPONENTS
				SAVE OBJECTS' LABEL NUMBER & COUNTS AT OBJ, OBJ_COUNT
				ASSIGN UNIQUE COLOR TO FILTERED COMPONENTS
				USER VARIABLE	:	MIN_AREA
			*/

			for (i = 1; i < components; i++)
			{
				if (stats.at<int>(i, CC_STAT_AREA) < MIN_AREA)
				{
					colors[i] = Vec3b(0, 0, 0);
					continue;
				}
				obj_count++;
				obj[obj_count] = i;
				colors[i] = Vec3b(79 * obj_count % 256, 101 * obj_count % 256, 129 * obj_count % 256);
			}
			Mat img_color = Mat::zeros(image.size(), CV_8UC3);
			printf("Objects found : %d\n", obj_count);
			
			for (y = 0; y < HEIGHT; y++)
			{
				for (x = 0; x < WIDTH; x++)
				{
					int label = image_label.at<int>(y, x);
					CV_Assert(0 <= label && label <= components);
					if (label != obj[0])
						img_color.at<Vec3b>(y, x) = colors[label];
				}	
			}
			/*
				VISUALIZE THE CENTER POSITION OF EACH COMPONENT WITH WHITE +
				PRINT EACH COMPONENT'S SIZE, CENTER POSITION DATA
				VISUALIZE FILTERED IMAGE WITH EACH UNIQUE COLOR
			*/
			for (i = 1; i <= obj_count; i++)
			{
				img_color.at<Vec3b>(cens.at<double>(obj[i], 1), cens.at<double>(obj[i], 0)) = Vec3b(255, 255, 255);
				img_color.at<Vec3b>(cens.at<double>(obj[i], 1)-1, cens.at<double>(obj[i], 0)) = Vec3b(255, 255, 255);
				img_color.at<Vec3b>(cens.at<double>(obj[i], 1), cens.at<double>(obj[i], 0)-1) = Vec3b(255, 255, 255);
				img_color.at<Vec3b>(cens.at<double>(obj[i], 1)+1, cens.at<double>(obj[i], 0)) = Vec3b(255, 255, 255);
				img_color.at<Vec3b>(cens.at<double>(obj[i], 1), cens.at<double>(obj[i], 0)+1) = Vec3b(255, 255, 255);
			}
			for (i = 1; i <= obj_count; i++)
			{
				std::cout << "		CENTER	"<< i <<"  = (" << cens.at<double>(obj[i], 0) << "," << cens.at<double>(obj[i], 1) << ")" << std::endl;
				std::cout << "		AREA	" << i << "  = " << stats.at<int>(obj[i], CC_STAT_AREA) <<  std::endl;
				std::cout << std::endl;
			}
			imshow("Labeled image", img_color);
			
			rs2_release_frame(frame);
		}
		rs2_release_frame(frames);
	}
	rs2_pipeline_stop(pipeline, &e);
	check_error(e);

	rs2_delete_pipeline_profile(pipe);
	rs2_delete_config(config);
	rs2_delete_pipeline(pipeline);
	rs2_delete_device(dev);
	rs2_delete_device_list(device_list);
	rs2_delete_context(ctx);

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



