#include <iostream>
#include <ros/ros.h>
#include "ros_topic_and_messages_pkg/SensorData.h"
#include <random>

void publish_sensor_data() {
    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ros_topic_and_messages_pkg::SensorData>("sensor_topic", 10);
    ros::Rate rate(1);  // 1 Hz

    while (ros::ok()) {
        ros_topic_and_messages_pkg::SensorData sensor_data;
        sensor_data.temperature = std::uniform_real_distribution<double>(20.0, 30.0)(std::default_random_engine());
        sensor_data.humidity = std::uniform_real_distribution<double>(40.0, 60.0)(std::default_random_engine());

        ROS_INFO("Publishing sensor data: Temperature=%f, Humidity=%f", sensor_data.temperature, sensor_data.humidity);
        pub.publish(sensor_data);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    try {
        publish_sensor_data();
    } catch (ros::ROSInterruptException& e) {
        // Handle exception
    }
    return 0;
}
