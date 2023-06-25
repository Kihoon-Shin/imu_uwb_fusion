#include <ros/ros.h>
#include <iostream>
#include <vector>

#include "imu_uwb_fusion.h"
#include "imu_uwb_fusion/UwbMsg.h"


class uwbPlot
{
public:
    uwbPlot()
    {
        // 퍼블리쉬 할 토픽 선언
        pub_ = n_.advertise<imu_uwb_fusion::UwbMsg>("/uwb_plot", 1000);

        // 서브스크라이브 할 토픽 선언
        sub_ = n_.subscribe("/uwb", 1, &uwbPlot::callback, this);
    }

    void callback(const imu_uwb_fusion::UwbMsg& input)
    {
        imu_uwb_fusion::UwbMsg output;
        output.pos_x = input.pos_x - 2.4;
        output.pos_y = input.pos_y - 0.6;
        output.pos_z = input.pos_z;

        pub_.publish(output);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_plot");
    uwbPlot uwbPlotNode;
    ros::spin();
    return 0;
}