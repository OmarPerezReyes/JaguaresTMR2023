#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <signal.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c

#define KEYCODE_T 0x74

#define KEYCODE_COMMA 0x2c

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

void keyControl(){

    int speed;
    int steering;
    int kfd;
    char c;

    autominy_msgs::SteeringPWMCommand steeringMsg;
    autominy_msgs::SpeedPWMCommand speedMsg;

    if(first_time){
        kfd=0;    
        speed = 0;
        steering = 1500;
        steeringMsg.value = static_cast<int16_t>(steering);
        pubSteering.publish(steeringMsg);

        // This MUST be here
        //mssleep(10); 

        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
    }
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
        perror("read():");
        exit(-1);
    }
    //intervals:
    // Steering_pwm [950, 2150], left-right
    // Speed_pwm [-1000, 1000] backwards forward 

    switch(c)  
    {
        case KEYCODE_I:
            if (speed > -100 && speed < 100){
                if (speed < 0){  
                    speed = 0;
                    speedMsg.value = speed;
                    pubSpeed.publish(speedMsg);
                    //mssleep(100); 
                }
                pubSpeed.publish(speedMsg);
                speed += 10;
                speedMsg.value = speed;
                pubSpeed.publish(speedMsg);
            } 
        break;

    case KEYCODE_K:
            speed = 0;
            speedMsg.value = speed;
            pubSpeed.publish(speedMsg);
        break;

    case KEYCODE_J:
        if (steering > 1000){
            steering -= 100.0;  
            steeringMsg.value = steering;
            pubSteering.publish(steeringMsg);
        }        
        break;

    case KEYCODE_L:
        if (steering < 2000){
            steering += 100.0;  
            steeringMsg.value = steering;
            pubSteering.publish(steeringMsg);
        } 
        break;

    case KEYCODE_COMMA:
        if (speed > -100 && speed < 100){
            if (speed > 0){  
                speed = 0;
                speedMsg.value = speed;
                pubSpeed.publish(speedMsg);
                //mssleep(100); 
            } 
            speed -= 10;
            speedMsg.value = speed;
            pubSpeed.publish(speedMsg);
        } 
        break;
    case KEYCODE_T:
            first_time = false;
        break;
    }
    //loop_rate.sleep(); 
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);
    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);

    signal(SIGINT, mySigintHandler);
    
    ros::spin();

    return 0;
}
