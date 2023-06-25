#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include "imu_uwb_fusion.h"
#include "imu_uwb_fusion/UwbMsg.h"


class resultPublish
{
public:
    resultPublish()
    {
        // 퍼블리쉬 할 토픽 선언
        pub_ = n_.advertise<geometry_msgs::Point>("/result", 1000);

        // 서브스크라이브 할 토픽 선언
        sub_ = n_.subscribe("/traj", 1, &resultPublish::callback, this);
    }

    void callback(const nav_msgs::Path& input)
    {
        geometry_msgs::Point output;
        output.x = input.poses.pose.position.x;
        output.y = input.poses.pose.position.y;

        pub_.publish(output);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "result_publish");
    resultPublish resultNode;
    ros::spin();
    return 0;
}