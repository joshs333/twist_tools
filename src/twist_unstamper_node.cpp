/**
 * @brief Node that inputs a stamped twist message and outputs a regular twist message
 * @author Joshua Spisak <joshs333@live.com>
 * @date October 5, 2020
 * @license MIT
 **/
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>


namespace twist_tools {

/**
 * @brief node that subscribes to a twist stamped and outputs a twist message
 **/
class TwistUnstamperNode {
public:
    /**
     * @brief parameters for creating node
     **/
    struct Params {
        //! Input twist stamped topic
        std::string input_twist_stamped = "/twist_stamped_in";
        //! Output twist topic
        std::string output_twist = "/twist_out";
    };

    /**
     * @brief creates node
     * @param nh nodehandle to use to create topics, etc..
     * @param params general params to generate node with
     **/
    TwistUnstamperNode(ros::NodeHandle& nh, Params& params) {
        input_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(params.input_twist_stamped, 1, &TwistUnstamperNode::twistCB, this);
        output_pub_ = nh.advertise<geometry_msgs::Twist>(params.output_twist, 1, true);
    }

    /**
     * @brief callback for twist messages
     * @param input the input twist message
     **/
    void twistCB(const geometry_msgs::TwistStamped::ConstPtr& input) {
        output_pub_.publish(input->twist);
    } 

private:
    //! Input Sub
    ros::Subscriber input_sub_;

    //! Output Publisher
    ros::Publisher output_pub_;

}; // TwistUnstamperNode

}; /* namespace twist_tools */

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_stamper_node");

    ros::NodeHandle nh("~");

    twist_tools::TwistUnstamperNode::Params params;

    nh.getParam("input_twist_stamped", params.input_twist_stamped);
    nh.getParam("output_twist", params.output_twist);

    twist_tools::TwistUnstamperNode node(nh, params);

    ros::spin();

    return 0;
}