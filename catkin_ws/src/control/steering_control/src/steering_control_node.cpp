#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <signal.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>

//ROS Publishers
ros::Publisher pubSpeed;
ros::Publisher pubSteering;

bool first_time = true;

void mySigintHandler(int sig)
{
    autominy_msgs::SpeedPWMCommand speedMsg;
    speedMsg.value = 0;

    pubSpeed.publish(speedMsg);
    ros::shutdown();
}

void steering_calculate(float obs_rho, float obs_theta, float best_rho, float best_theta)
{
    //ros::Rate loop_rate(10);  // 1/10 Secs Check loop_rate of function ros spin

    int speed;
    int steering;
    double prev_error_theta;
    double diff_error;
    double error_theta;
   
    autominy_msgs::SteeringPWMCommand steeringMsg;
    autominy_msgs::SpeedPWMCommand speedMsg;

    if (ros::ok()) {
        speed = 45;  
    }

    if (first_time) {
        // Run fist_time program initalize all variables
        steering = 1470;
        steeringMsg.value = static_cast<int16_t>(steering);
        pubSteering.publish(steeringMsg);
            
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);

        first_time = false;
        
        prev_error_theta = 0;
        diff_error = 0;
    }

    if (obs_theta != 0 && best_theta != 0) {
        error_theta = obs_theta - best_theta;

        if (error_theta > M_PI) error_theta -= 2*M_PI; 
        if (error_theta < -M_PI) error_theta += 2*M_PI; 

        diff_error = error_theta - prev_error_theta; 

        if (diff_error > M_PI) diff_error -= 2*M_PI; 
        if (diff_error < -M_PI) diff_error += 2*M_PI; 

        double error_rho = obs_rho - best_rho;

        double diff = (obs_rho - 210) * 5;

        prev_error_theta = error_theta;

        steering = 1440 + diff;

        if(steering < 880){ //880 
           steering = 880;
        }
        else{
            if(steering > 2060) {
                steering = 2060; 
            }
        }
    } else if (obs_theta == 0 && best_theta != 0) { 
        steering = 1470;
    }


    std::cout << "best_rho "<< best_rho << " best_theta " << best_theta << std::endl; 
    std::cout << "obs_rho "<< obs_rho << " obs_theta " << obs_theta << std::endl;        
    std::cout << "error_theta "<< error_theta << " steering " << steering << std::endl; 
    std::cout.flush();

    if (steeringMsg.value != steering){
        
        steeringMsg.value = steering;
        pubSteering.publish(steeringMsg);
    }
        
    //loop_rate.sleep(); check ros sleep with ros spin()
}

void laneCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //steering_calculate(float obs_rho, float obs_theta, float best_rho, float best_theta) 
    steering_calculate(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "steering_control_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/lane_right", 1, laneCallback);

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);
    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);

    // ros::Subscriber speed = n.subscribe("/sensors/arduino/steering_angle", 10, steeringCallback); A 

    signal(SIGINT, mySigintHandler);
    
    ros::spin();

    return 0;
}
