/**
 * @brief Node muxes between stamped twist topics
 * @author Joshua Spisak <joshs333@live.com>
 * @date October 5, 2020
 * @license MIT
 **/
// ros
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

// application
#include "msg_mux/msg_mux.hpp"

typedef twist_tools::MsgMux<geometry_msgs::TwistStamped> TwistStampedMux;

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_mux_node");

    ros::NodeHandle nh("~");

    TwistStampedMux::Params params;
    nh.getParam("input_topics",                     params.input_topics);
    nh.getParam("default_input",                    params.default_input);
    nh.getParam("output_topic",                     params.output_topic);
    nh.getParam("status_topic",                     params.status_topic);
    nh.getParam("set_input_service",                params.set_input_service);
    nh.getParam("iterate_input_service",            params.iterate_input_service);
    nh.getParam("reverse_iterate_input_service",    params.reverse_iterate_input_service);
    nh.getParam("allow_new_topics",                 params.allow_new_topics);

    TwistStampedMux node(nh, params);

    ros::spin();

    return 0;
}