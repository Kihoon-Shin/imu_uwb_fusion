#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <thread>
#include <mutex>
#include "imu_uwb_fusion.h"

#include "imu_uwb_fusion/UwbMsg.h"

using namespace std;

int imu_freq = 100; //imu frequency is location frequency

nav_msgs::Path path; //result path

vector<sensor_msgs::ImuConstPtr> imu_buffer;
//vector<sensor_msgs::NavSatFixConstPtr> gps_buffer;
vector<imu_uwb_fusion::UwbMsg::ConstPtr> uwb_buffer;
mutex imu_mtx, uwb_mtx;

// vector<imu_uwb_fusion::UwbMsg::Ptr> SMA_buffer;
// double length = 5;
// double sum_x, sum_y, sum_z = 0;
// int cnt = 0;

Fusion::ImuUwbFusion imu_uwb_fuser; // fuser object
ros::Publisher traj_puber;
ros::Publisher result_puber;
bool initialized = false;
Fusion::ImuData<double> last_uwb_imu;     //last interpolated imu data at uwb time
Fusion::ImuData<double> last_imu;         //last imu data for predict
Fusion::State<double> last_updated_state; //last updated state by uwb

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    unique_lock<mutex> lock(imu_mtx);
    imu_buffer.push_back(msg);
}

void uwbCallback(const imu_uwb_fusion::UwbMsg::ConstPtr& msg)
{
    unique_lock<mutex> lock(uwb_mtx);
    uwb_buffer.push_back(msg);
}

// void uwbCallback(const imu_uwb_fusion::UwbMsg::ConstPtr& msg)
// {
    
//     SMA_buffer.push_back(msg);
//     imu_uwb_fusion::UwbMsg tmp_msg;

//     tmp_msg.header = msg->header;
//     tmp_msg.pos_x = msg->pos_x;
//     tmp_msg.pos_y = msg->pos_y;
//     tmp_msg.pos_z = msg->pos_z;
    
//     sum_x += msg->pos_x;
//     sum_y += msg->pos_y;
//     sum_z += msg->pos_z;
//     cout << "current cnt is " << cnt << endl;

//     if (cnt >= length)
//     {
//         cout << "cnt - length " << cnt - length << endl;
//         cout << SMA_buffer[cnt - length]->pos_x << endl;

//         sum_x -= SMA_buffer[cnt - length]->pos_x;
//         sum_y -= SMA_buffer[cnt - length]->pos_y;
//         sum_z -= SMA_buffer[cnt - length]->pos_z;

//     }
//     if (cnt >= length-1)
//     {
//         cout << "Your SMA x: " << (sum_x / (double) length) << endl;
//         tmp_msg.pos_x = (sum_x / (double) length);
//         cout << "Your SMA y: " << (sum_y / (double) length) << endl;
//         tmp_msg.pos_y = (sum_y / (double) length);
//         cout << "Your SMA z: " << (sum_z / (double) length) << endl;
//         tmp_msg.pos_z = (sum_z / (double) length);

//         const imu_uwb_fusion::UwbMsg::ConstPtr tmp_ptr(&tmp_msg);
        
//         unique_lock<mutex> lock(uwb_mtx);
//         uwb_buffer.push_back(tmp_ptr);
//     }
//     cnt++;

//         // unique_lock<mutex> lock(uwb_mtx);
//         // uwb_buffer.push_back(msg);

// }


void interpolateImuData(const sensor_msgs::ImuConstPtr &first_data, const sensor_msgs::ImuConstPtr &second_data, double cur_stamp, sensor_msgs::Imu &inter_data)
{
    // linear_interpolation (https://blog.naver.com/aorigin/220947541918)
    double first_stamp = first_data->header.stamp.toSec();
    double second_stamp = second_data->header.stamp.toSec();
    double scale = (cur_stamp - first_stamp) / (second_stamp - first_stamp);
    inter_data = *first_data;
    inter_data.angular_velocity.x = scale * (second_data->angular_velocity.x - first_data->angular_velocity.x) + first_data->angular_velocity.x;
    inter_data.angular_velocity.y = scale * (second_data->angular_velocity.y - first_data->angular_velocity.y) + first_data->angular_velocity.y;
    inter_data.angular_velocity.z = scale * (second_data->angular_velocity.z - first_data->angular_velocity.z) + first_data->angular_velocity.z;
    inter_data.linear_acceleration.x = scale * (second_data->linear_acceleration.x - first_data->linear_acceleration.x) + first_data->linear_acceleration.x;
    inter_data.linear_acceleration.y = scale * (second_data->linear_acceleration.y - first_data->linear_acceleration.y) + first_data->linear_acceleration.y;
    inter_data.linear_acceleration.z = scale * (second_data->linear_acceleration.z - first_data->linear_acceleration.z) + first_data->linear_acceleration.z;
}

Fusion::ImuData<double> fromImuMsg(const sensor_msgs::Imu &msg)
{
    Fusion::ImuData<double> imu_data;
    imu_data.stamp = msg.header.stamp.toSec();
    imu_data.gyr[0] = msg.angular_velocity.x;
    imu_data.gyr[1] = msg.angular_velocity.y;
    imu_data.gyr[2] = msg.angular_velocity.z;
    imu_data.acc[0] = msg.linear_acceleration.x;
    imu_data.acc[1] = msg.linear_acceleration.y;
    imu_data.acc[2] = msg.linear_acceleration.z;

    return move(imu_data);
}

Fusion::UwbData<double> fromUwbMsg(const imu_uwb_fusion::UwbMsg &msg)
{
    Fusion::UwbData<double> uwb_data;
    uwb_data.data[0] = msg.pos_x;
    uwb_data.data[1] = msg.pos_y;
    // uwb_data.data[2] = msg.pos_z;
    uwb_data.data[2] = 0.2;
    uwb_data.cov.setIdentity();
    uwb_data.cov(0, 0) = 0.00002; // need to fix it
    uwb_data.cov(1, 1) = 0.00002;
    uwb_data.cov(2, 2) = 0.00002;
    return move(uwb_data);
}

void pubResult()
{
    if (traj_puber.getNumSubscribers() != 0)
    {
        Fusion::State<double> result = imu_uwb_fuser.getNominalState();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = uwb_buffer[0]->header.stamp;
        pose.pose.position.x = result.p[0];
        pose.pose.position.y = result.p[1];
        pose.pose.position.z = result.p[2];
        pose.pose.orientation.w = result.q.w();
        pose.pose.orientation.x = result.q.x();
        pose.pose.orientation.y = result.q.y();
        pose.pose.orientation.z = result.q.z();
        path.poses.push_back(pose);
        path.header = pose.header;
        traj_puber.publish(path);
        result_puber.publish(pose);
    }
}

void processThread()
{
    ros::Rate loop_rate(imu_freq);
    while (ros::ok())
    {
        unique_lock<mutex> imu_lock(imu_mtx);
        unique_lock<mutex> uwb_lock(uwb_mtx);

        // if no data, no need update state
        if (!imu_buffer.size() && !uwb_buffer.size())
        {
            ROS_INFO_THROTTLE(10, "wait for uwb or imu msg ......");
            imu_lock.unlock();
            uwb_lock.unlock();
            loop_rate.sleep();
            continue;
        }

        // init correlative param
        if (!initialized)
        {
            // wait for enough sensor data to init
            if (!imu_buffer.size() || !uwb_buffer.size())
            {
                ROS_INFO_THROTTLE(10, "wait for uwb or imu msg ......");
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }

            // use imu datas at start to initial imu pose
            vector<Fusion::ImuData<double>> imu_datas;
            for (auto &imu_msg : imu_buffer)
            {
                imu_datas.push_back(fromImuMsg(*imu_msg));
            }
            imu_uwb_fuser.imuInit(imu_datas);

            // set reference ll for convert ll to enu frame
            imu_uwb_fuser.cfgRefUwb(uwb_buffer[0]->pos_x, uwb_buffer[0]->pos_y, uwb_buffer[0]->pos_z);

            // interpolate first imu data at first uwb time
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.begin() == iter || imu_buffer.end() == iter) //cant find imu data before or after gps data
            {
                if (imu_buffer.begin() == iter)
                    uwb_buffer.erase(uwb_buffer.begin()); //no imu data before first gps data, cant interpolate at gps stamp
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            sensor_msgs::Imu inter_imu;
            double cur_stamp = uwb_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);

            //record last gps frame time and interpolated imu data
            last_uwb_imu = fromImuMsg(inter_imu);
            last_uwb_imu.stamp = cur_stamp;
            last_imu = last_uwb_imu;
            last_updated_state = imu_uwb_fuser.getState();

            // delete old imu datas and gps data
            imu_buffer.erase(imu_buffer.begin(), iter);
            uwb_buffer.erase(uwb_buffer.begin());

            imu_lock.unlock();
            uwb_lock.unlock();
            loop_rate.sleep();

            initialized = true;

            continue;
        }

        // use imu predict location for increase locate frequency
        // actual state no change
        for (auto &imu_msg : imu_buffer)
        {
            Fusion::ImuData<double> cur_imu = fromImuMsg(*imu_msg);
            if (cur_imu.stamp > last_imu.stamp)
            {
                imu_uwb_fuser.updateNominalState(last_imu, cur_imu);
                last_imu = cur_imu;
            }
        }

        // use uwb data to update state
        if (uwb_buffer.size() != 0)
        {
            // if (uwb_buffer.front()->status.status != 2)
            // {
            //     cout << "gps data is bad !!!" << endl;
            //     gps_buffer.erase(gps_buffer.begin());
            //     imu_lock.unlock();
            //     gps_lock.unlock();
            //     loop_rate.sleep();
            //     continue;
            // }

            // recover to last updated state for imu predict again
            imu_uwb_fuser.recoverState(last_updated_state);

            // collect imu datas during two neighbor gps frames
            vector<Fusion::ImuData<double>> imu_datas(0);
            // search first imu data after gps data
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.end() == iter) // no imu data after first gps data, wait for new imu data
            {
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            assert(imu_buffer.begin() != iter);
            // add last gps_imu data (interpolated data at gps time)
            imu_datas.push_back(last_uwb_imu);
            // add imu data between last gps_imu data and current gps_imu data
            for (auto tmp_iter = imu_buffer.begin(); tmp_iter != iter; tmp_iter++)
                imu_datas.push_back(fromImuMsg(*(*tmp_iter)));
            // add current gps_imu data
            sensor_msgs::Imu inter_imu;
            double cur_stamp = uwb_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);
            Fusion::ImuData<double> cur_uwb_imu = fromImuMsg(inter_imu);
            cur_uwb_imu.stamp = cur_stamp;
            imu_datas.push_back(cur_uwb_imu);

            // generate uwb data
            Fusion::UwbData<double> uwb_data = fromUwbMsg(*uwb_buffer[0]);

            // update state (core)
            imu_uwb_fuser.uwbUpdate(uwb_data, imu_datas);

            // update last data
            last_uwb_imu = cur_uwb_imu;
            last_imu = last_uwb_imu;

            // delete old data
            uwb_buffer.erase(uwb_buffer.begin());
            imu_buffer.erase(imu_buffer.begin(), iter);

            // update last state
            last_updated_state = imu_uwb_fuser.getState();
        }

        // publish result
        pubResult();

        imu_lock.unlock();
        uwb_lock.unlock();
        loop_rate.sleep();
        continue;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_uwb_fusion_node");
    ros::NodeHandle nh, ph("~");

    // load imu param and config fuser
    double sigma_an, sigma_wn, sigma_aw, sigma_ww;
    if (!ph.getParam("sigma_an", sigma_an) || !ph.getParam("sigma_wn", sigma_wn) || !ph.getParam("sigma_aw", sigma_aw) || !ph.getParam("sigma_ww", sigma_ww))
    {
        cout << "please config imu param !!!" << endl;
        return 0;
    }
    imu_uwb_fuser.cfgImuVar(sigma_an, sigma_wn, sigma_aw, sigma_ww);

    // load imu freqency param
    if (!ph.getParam("imu_freq", imu_freq))
    {
        cout << "no config imu_freq param, use default 100 !" << endl;
    }

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imuCallback);
    ros::Subscriber uwb_sub = nh.subscribe<imu_uwb_fusion::UwbMsg>("/uwb_filtered", 1000, uwbCallback);
    traj_puber = nh.advertise<nav_msgs::Path>("traj", 1);
    result_puber = nh.advertise<geometry_msgs::PoseStamped>("result", 1);

    // start process
    thread process_thread(processThread);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    process_thread.join();

    return 0;
}
