#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h" 

#include <opencv2/opencv.hpp>

bool first_time = true;
bool first_rectangles = true;

std::vector<cv::Rect> windows;
    
ros::Publisher pub_lane;

std::vector<cv::Point2f> slidingWindow(cv::Mat image, cv::Mat windowed, std::vector<cv::Rect> &windows)
{

    std::vector<cv::Point2f> points;
    const cv::Size imgSize = image.size();

    for (int i = 0; i < 5; i++) {

       float currentX = windows.at(i).x + windows.at(i).width * 0.5f;

       cv::rectangle(windowed, cv::Rect(windows.at(i).x, windows.at(i).y, windows.at(i).width, windows.at(i).height), cv::Scalar(255, 255, 0), 1, 8, 0); 

        //std::cout << rect.x << " " << rect.y << " " << rect.width << " " <<  rect.height << std::endl; 
        //std::cout.flush();                        
        cv::Mat roi = image(windows.at(i)); //Extract region of interest         
        cv::Mat locations;   // output, locations of non-zero pixels 

        cv::findNonZero(roi, locations); //Get all non-black pixels. All are white in our case         

        float avgX = 0.0f;

        // std::cout << "i: " << i << " locations.elemSize() " <<  locations.elemSize() << std::endl;
        if (locations.elemSize() > 0){

        for (int j = 0; j < locations.elemSize(); ++j) //Calculate average X position         
        {
            float x = locations.at<cv::Point>(j).x;
            avgX += windows.at(i).x + x; // x-coordinate relative to the original image
        }

        avgX = locations.empty() ? currentX : avgX / locations.elemSize();

        cv::Point point(avgX, windows.at(i).y + windows.at(i).height * 0.5f);
        points.push_back(point);

        //Move x position
        windows.at(i).x = (int)round(avgX - windows.at(i).width *.5);

        if (i < windows.size() - 1){
            if (windows.at(i).x + windows.at(i).width < windows.at(i + 1).x ||  windows.at(i).x > windows.at(i + 1).x + round(windows.at(i + 1).width)){
                //pts.erase(pts.begin() + i);
                windows.at(i).x = windows.at(i + 1).x;
                points.pop_back();
                cv::Point new_point(
                windows.at(i + 1).x + round(windows.at(i + 1).width * 0.5f),
                windows.at(i).y + windows.at(i).height * 0.5f);
                points.push_back(new_point);     
            }
        }

        if (i == windows.size() - 1){
            if (windows.at(i).x + windows.at(i).width < windows.at(i - 1).x ||  windows.at(i).x > windows.at(i - 1).x + round(windows.at(i - 1).width)){
                float avgX = 0.0f;

                for (int j = 0; j < windows.size() - 1; ++j){
                    avgX += windows.at(i).x; 
                }

                avgX = avgX / (windows.size() - 1);
                windows.at(i).x = avgX; //windows.at(i - 1).x;
                points.pop_back();
                cv::Point new_point(windows.at(i).x, windows.at(i).y + windows.at(i).height * 0.5f);
                points.push_back(new_point);
            }  

        }

        //std::cout << " windows.at(i).x " << windows.at(i).x << " point.x " <<  point.x << " currentX " << currentX <<  std::endl;
        //std::cout << " windows.at(i).width " << windows.at(i).width <<  std::endl;        
        //std::cout.flush();               
        //Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix         
        if (windows.at(i).x < 0)
            windows.at(i).x = 0;
        if (windows.at(i).x + windows.at(i).width >= imgSize.width)
            windows.at(i).x = imgSize.width - windows.at(i).width - 1;

    } // if locations.elemSize() > 0

    //std::cout << " Overflow windows.at(i).x " << windows.at(i).x <<  std::endl;
    //std::cout.flush();        
    }

    cv::imshow("dawsd", windowed);
    cv::waitKey(1);

    return points;
}

void process_img(cv::Mat orig_image, float &obs_rho_pub, float &obs_theta_pub, float &best_rho_pub ,float &best_theta_pub, cv::Mat &out_image) {

    double obs_theta = 0;
    double obs_rho = 0;
    double best_theta = 0.896055;
    double best_rho =  330.932;

    cv::Mat processed;
    cv::Mat transformed(480, 640, CV_8UC3); //Destination for warped image                  
    cv::Mat windowed(480, 640, CV_8UC3);
    cv::Mat orig = orig_image.clone(); 
  
    
    if (first_rectangles) {
        int y_rect = 384;
        int h_rect = 96;

        for (int i = 0; i < 5; i++) {
            windows.push_back(cv::Rect(440, y_rect, 200, h_rect));                   
            y_rect = y_rect - h_rect;
            //std::cout << i << std::endl;
        }
        first_rectangles = false;
    }

    // Extract ROI myROI(x, y, width, height)
    // HHAA: ROI dimensions MUST be tuned.
    //cv::Rect myROI(100, 239, 540, 240); // pt1(x,y), pt2(x + width, y + height)     
    
// while start 
    cv::Rect myROI(0, 200, 639, 279);
    cv::Mat image_roi = orig(myROI);  

    //    if (contador == 3){ 

    // Extract ROI myROI(x, y, width, height)
    // HHAA: ROI dimensions MUST be tuned.
    //    cv::Rect myROI(100, 239, 540, 240); // pt1(x,y), pt2(x + width, y + height)      
    //Convert to gray 
    cv::Mat image_grey;
    cv::cvtColor(image_roi, image_grey, cv::COLOR_RGB2GRAY);

    // Extract white info 
    cv::Mat maskWhite;
    // HHAA: White threshold MUST be tuned.
    cv::inRange(image_grey, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), maskWhite);            
    cv::bitwise_and(image_grey, maskWhite, processed); // Extract what matches   

    // Blur the image a bit so that gaps are smoother 
    // HHAA: Kernel size could be fine-tuned.
    const cv::Size kernelSize = cv::Size(9, 9);
    cv::GaussianBlur(processed, processed, kernelSize, 0);

    //Try to fill the gaps 
    // HHAA: Is this block necessary?
    cv::Mat kernel = cv::Mat::ones(15, 15, CV_8U);
    cv::dilate(processed, processed, kernel);
    cv::erode(processed, processed, kernel);
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);

    // Keep only what's above 100 value, other is then black 
    // HHAA: White threshold MUST be tuned.     
    const int thresholdVal = 100;
    cv::threshold(processed, processed, thresholdVal, 255, cv::THRESH_BINARY);

    // Transformacion homografica
    int x1 = 150;
    int x2 = 650;
    int x3 = 560;
    int x4 = 190;
    int y1 = 160;
    int y2 = 80;

    cv::Point2f srcVertices[4]; 
    srcVertices[0] = cv::Point(x1, y1);
    srcVertices[1] = cv::Point(x2, y1);
    srcVertices[2] = cv::Point(x3, y2);
    srcVertices[3] = cv::Point(x4, y2);       

    // Destination vertices. Output is 640 by 480px       
    cv::Point2f dstVertices[4];
    dstVertices[0] = cv::Point(0, 0);
    dstVertices[1] = cv::Point(640, 0);
    dstVertices[2] = cv::Point(640, 480);
    dstVertices[3] = cv::Point(0, 480);

    // Prepare matrix for transform and get the warped image 
    cv::Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
    
    //For transforming back into original image space 
    cv::Mat invertedPerspectiveMatrix;
    cv::invert(perspectiveMatrix, invertedPerspectiveMatrix);

    cv::warpPerspective(processed, transformed, perspectiveMatrix, transformed.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);      
    
    transformed.copyTo(windowed);

    /*
    int y_rect = 384;
    int h_rect = 96;

    for (int i = 0; i < 5; i++){
        windows.push_back(cv::Rect(440, y_rect, 200, h_rect));                   
        y_rect = y_rect - h_rect;
    }
    */
    std::vector<cv::Point2f> pts = slidingWindow(transformed, windowed, windows);
    
    if (pts.size() > 0){

    std::vector<cv::Point2f> outPts;
    perspectiveTransform(pts, outPts, invertedPerspectiveMatrix);

    cv::Point p1 = outPts[0];
    cv::Point p2 = outPts[outPts.size() - 1];  
    //Línea trazada sobre el borde del carril
    cv::line(image_roi, p1, p2, cv::Scalar(255, 0, 100), 3);

    double m = (p2.y - p1.y)/ (double) (p2.x - p1.x);  
    obs_theta =  atan(m);

    if (obs_theta < 0){

    obs_theta += M_PI*2; 
    }

    std::cout.flush(); 
    cv::Point mid;

    if (first_time){

        obs_theta = best_theta; 
        obs_rho = best_rho; 

        first_time = false;
    }  else {

        mid.x = round((p2.x - p1.x) / 2.0) + p1.x;   
        mid.y = round((p2.y - p1.y) / 2.0) + p1.y;   

        obs_rho = sqrt((pow(320 - mid.x, 2) + pow(279 - mid.y, 2)));

    }
        //linea que va desde el origen hasta el punto medio de la línea sobre el carril
        cv::line(image_roi, cv::Point(mid.x, mid.y), cv::Point(320, 279), cv::Scalar(255, 0, 255), 3, cv::LINE_AA);  
    } 
    obs_rho_pub = obs_rho;
    obs_theta_pub = obs_theta;
    best_rho_pub = best_rho;
    best_theta_pub = best_theta;

    out_image = image_roi;
}

// Messages are passed to a callback function (here)
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{

    //sensor_msgs::Image msg;
    // ROS_INFO return the data that recived
    // IMPRIMIR LO DE ARRIBA, VER QUE TIPO DE DATOS SON sensor_msgs::ImageConstPtr& msg

    //msg = *msg_const_ptr;
    cv::Mat color(cv::Size(640, 480), CV_8UC3, (unsigned char*)msg->data.data(), cv::Mat::AUTO_STEP);
    //std::cout << std::endl << "image recieved" << std::endl;
   
    float obs_rho_pub, obs_theta_pub, best_rho_pub, best_theta_pub;  
    cv::Mat result;

    process_img(color, obs_rho_pub, obs_theta_pub, best_rho_pub, best_theta_pub, result);
    cv::imshow("img", result);
    cv::waitKey(1);
    
    std_msgs::Float32MultiArray lane_msg;

    lane_msg.data.push_back(obs_rho_pub);
    lane_msg.data.push_back(obs_theta_pub);
    lane_msg.data.push_back(best_rho_pub);
    lane_msg.data.push_back(best_theta_pub);

    pub_lane.publish(lane_msg);
}

int main(int argc, char **argv)
{
    /*
    The ros::init() function needs to see argc and argv so that it can perform
    any ROS arguments and name remapping that were provided at the command line. 
    The third argument to init() is the name of the node.
    */
    ros::init(argc, argv, "lane_detector_node");

    /*
    NodeHandle is the main access point to communications with the ROS system.
    The first NodeHandle constructed will fully initialize this node, and the last
    NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /*
    The subscribe() call is how you tell ROS that you want to receive messages on a given topic.  
    This invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing 
    this callback will automatically be unsubscribed from this topic.

    The second parameter to the subscribe() function is the size of the message queue (1 - 1000).
    The third is the callBack function

    */

    // subscribe() returns a Subscriber object that you
    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);

    pub_lane = n.advertise<std_msgs::Float32MultiArray>("/lane_right", 1);
 
    // ros::spin() will enter a loop, pumping callbacks. 
    ros::spin(); 

    return 0;
}
