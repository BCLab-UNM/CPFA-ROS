#include "sbridge.h"

sbridge::sbridge(std::string publishedName) {

    ros::NodeHandle sNH;


    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);

    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);
    modeSubscriber = sNH.subscribe((publishedName + "/mode"), 1, &sbridge::modeHandler, this);

    float heartbeat_publish_interval = 2;
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);


     ROS_INFO("constructor");

}

void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    double left = (message->linear.x);
    double right = (message->angular.z);

    float turn = 0;
    float forward = 0;

    if (currentMode == 1) {
        forward = left*0.6;
        turn = right*1.4;
        if (forward >= 0.4){

            forward -= (abs(turn)/5);
        }

    }
    else
    {

        float linearVel = (left + right)/2;
        float angularVel = (right-left)/2;

        turn = angularVel/180;
        forward = linearVel/400;
        if (forward >= 150){

            forward -= (abs(turn)/5);
        }

        if (linearVel >= 0 && forward <= 0)
        {
            forward = 0;
        }
        if (linearVel <= 0 && forward >= 0)
        {
            forward = 0;
        }
    }

    if (fabs(forward) >= 0.8) {
        forward = forward/fabs(forward) * 0.8;
    }

    if (fabs(turn) >= 1.0) { //max value needs tuning
        turn = turn/fabs(turn) * 1.0;
    }




    velocity.linear.x = forward,
            velocity.angular.z = turn;
    skidsteerPublish.publish(velocity);
}

void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);

    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

void sbridge::modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
}

sbridge::~sbridge() {
}
