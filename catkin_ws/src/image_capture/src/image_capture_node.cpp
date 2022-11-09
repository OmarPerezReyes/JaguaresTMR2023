// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    //Get each frame
    ros::init(argc, argv, "image_capture"); 

    ros::NodeHandle n;

    ros::Publisher pub_img = n.advertise<sensor_msgs::Image>("/realsense/color_raw", 1);

    sensor_msgs::Image img;

    img.width = 640;

    img.height = 480;

    img.encoding = "bgr8";

    img.data.resize(640*480*3);    

    img.step = 3*640;

    ros::Rate loop(30);

    while(ros::ok()){
        frames = pipe.wait_for_frames();

        rs2::frame color_frame = frames.get_color_frame();
        //Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        // Display in a GUI
        //namedWindow("Display Image", WINDOW_AUTOSIZE );
        //imshow("Display Image", color);

        waitKey(10);

        img.header.stamp = ros::Time::now();

        // img.data = (unsigned char*)color_frame.get_data();
        memcpy(img.data.data(), color_frame.get_data(), 3*640*480);

        pub_img.publish(img);

        ros::spinOnce();

        loop.sleep();
    }

    return 0;

}
