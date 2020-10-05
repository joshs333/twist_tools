/**
 * @brief Node that inputs a regular twist message and outputs a stamped twist message
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
 * @brief node that subscribes to a twist and outputs a stamped twist message
 **/
class TwistStamperNode {
public:
    /**
     * @brief parameters for creating node
     **/
    struct Params {
        //! Input twist topic
        std::string input_twist = "/twist_in";
        //! Output twist stamped topic
        std::string output_twist_stamped = "/twist_stamped_out";
        //! Output stamp frame_id
        std::string stamp_frame_id = "/twist";
    };

    /**
     * @brief creates node
     * @param nh nodehandle to use to create topics, etc..
     * @param params general params to generate node with
     **/
    TwistStamperNode(ros::NodeHandle& nh, Params& params) {
        stamp_frame_id_ = params.stamp_frame_id;
        input_sub_ = nh.subscribe<geometry_msgs::Twist>(params.input_twist, 1, &TwistStamperNode::twistCB, this);
        output_pub_ = nh.advertise<geometry_msgs::TwistStamped>(params.output_twist_stamped, 1, true);
    }

    /**
     * @brief callback for twist messages
     * @param input the input twist message
     **/
    void twistCB(const geometry_msgs::Twist::ConstPtr& input) {
        geometry_msgs::TwistStamped out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = stamp_frame_id_;
        out_msg.twist = *input;
        output_pub_.publish(out_msg);
    } 

private:
    //! Frame id to put into output message
    std::string stamp_frame_id_;

    //! Input Sub
    ros::Subscriber input_sub_;

    //! Output Publisher
    ros::Publisher output_pub_;

}; // TwistStamperNode

}; /* namespace twist_tools */

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_stamper_node");

    ros::NodeHandle nh("~");

    twist_tools::TwistStamperNode::Params params;

    nh.getParam("input_twist", params.input_twist);
    nh.getParam("output_twist_stamped", params.output_twist_stamped);
    nh.getParam("stamp_frame_id", params.stamp_frame_id);

    twist_tools::TwistStamperNode node(nh, params);

    ros::spin();

    return 0;
}