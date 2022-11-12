#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <signal.h>
#include <termios.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

bool die = false;
bool first_time = true;

ros::Publisher pubSpeed;
ros::Publisher pubSteering;
autominy_msgs::SpeedPWMCommand speedMsg;
autominy_msgs::SteeringPWMCommand steeringMsg;


int keyControl(ros::Rate r){
    int kfd;
    int speed;
    int steering;
   
    char c;
    struct termios cooked, raw;


    if(first_time){

        speed=0;
        steering=1500;

        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);


        steeringMsg.value = steering;
        pubSteering.publish(steeringMsg);
        kfd = 0;
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Moving around:");
        puts("       w    ");
        puts("   a   s   d");
        puts("---------------------------");

        first_time=false;	
    }

    while(ros::ok() && !die){
      
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(c)  
        {
            case KEYCODE_W:
            if (speed > -100 && speed < 100){
                if (speed < 0){  
                    speed = 0;
                    speedMsg.value = speed;
                    pubSpeed.publish(speedMsg);
                }
                pubSpeed.publish(speedMsg);
                speed += 10;
                speedMsg.value = speed;
                pubSpeed.publish(speedMsg);
            }
            break;

            case KEYCODE_S:
                speed = 0;
                speedMsg.value = speed;
                pubSpeed.publish(speedMsg);
                break;

            case KEYCODE_A:
                if (steering > 1000){
                    steering -= 100.0;  
                    steeringMsg.value = steering;
                    pubSteering.publish(steeringMsg);
                }        
            break;

            case KEYCODE_D:
                if (steering < 2000){
                steering += 100.0;  
                steeringMsg.value = steering;
                pubSteering.publish(steeringMsg);
                } 
            break;
            default:
                   die=true;
        }

        r.sleep();     
    }

    return 0;

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    
    ros::Rate r(50); // 10 hz
  
    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);
    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);

    keyControl(r);
 
    speedMsg.value = 0;
    steeringMsg.value = 1500;

    pubSteering.publish(steeringMsg);

    r.sleep();   
    r.sleep();   
    r.sleep();   
    r.sleep();       

    pubSpeed.publish(speedMsg);

    r.sleep();       

    ros::shutdown();
   
    // ros::spin();
}
