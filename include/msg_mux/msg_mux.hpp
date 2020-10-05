/**
 * @brief templated class for a node that muxes between ros topics to a single output
 * @author Joshua Spisak <joshs333@live.com>
 * @date October 5, 2020
 * @license MIT
 * 
 * @details despite being in this twist_tools package this template is fairly versatile
 *  and can be used to mux between topics of any type. Life is good.
 **/
#ifndef TWIST_TOOLS_MSG_MUX
#define TWIST_TOOLS_MSG_MUX

// std tools
#include <map>
#include <vector>
#include <string>

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

// our messages / srvs
#include "twist_tools/RequestTopic.h"


namespace twist_tools {

/**
 * @brief a mux node that can mux any type
 **/
template<typename MuxedType>
class MsgMux {
public:
    /**
     * @brief parameters used to generate the MsgMux
     **/
    struct Params {
        //! Vector of default input topics
        std::vector<std::string> input_topics;
        //! Default input topic
        std::string default_input = "input";
        //! Output topic
        std::string output_topic = "output";

        //! Topic on which to publish status
        std::string status_topic = "status";
        //! Service used to set the input topic
        std::string set_input_service = "set_input";
        //! Service used to iterate through inputs
        std::string iterate_input_service = "iterate";
        //! Service used to reverse iterate through inputs
        std::string reverse_iterate_input_service = "reverse_iterate";
        //! Whether or not to allow new topics to be added if they are requested but not in input_topics
        bool allow_new_topics = true;

        //! Status publish period in ms
        double update_period_ms = 100;
    };

    /**
     * @brief creates a message mux from the provided params and a default nodehandle
     * @param params the parameters to use to generate topics & etc
     **/
    MsgMux(Params& params):
        nh_("~")
    {
        initNode(params);
    }  /* MsgMux::MsgMux() */

    /**
     * @brief creates a message mux from the provided params and the provided nodehandle
     * @param nh the nodehandle to use when creating publishers / subscribers
     * @param params the parameters to use to generate topics & etc
     **/
    MsgMux(ros::NodeHandle& nh, Params& params):
        nh_(nh)
    {
        initNode(params);
    } /* MsgMux::MsgMux() */

    /**
     * @brief callback for input messages
     * @param input_msg the message of the muxed type
     * @param topic_name the topic feeding into the callback
     **/
    void msgCB(
        const typename MuxedType::ConstPtr& input_msg,
        std::string topic_name
    ) {
        if ( topic_name != current_topic_ ) {
            return;
        }
        output_pub_.publish(input_msg);
    } /* MsgMux::msgCB() */

    /**
     * @brief service call to set the input topic
     * @param req ros service request
     * @param resp ros service response
     **/
    bool setInputService(
        twist_tools::RequestTopic::Request& req,
        twist_tools::RequestTopic::Response& resp
    ) {
        resp.success = false;
        resp.new_topic = false;
        resp.previous_topic = current_topic_;

        if (req.topic == "") {
            resp.success = true;
            ROS_WARN("Disabling output.");
            return true;
        }

        auto pub_it = input_subs_.find(req.topic);

        if (pub_it == input_subs_.end()) {
            if(req.topic == output_topic_) {
                ROS_ERROR("Topic requested [%s] is the output topic. Refusing to self-subscribe!", req.topic.c_str());
                return true;
            }
            resp.new_topic = true;
            if (!allow_new_topics_) {
                ROS_ERROR("Topic requested [%s] that isn't a current input topic, and no new topics are allowed.", req.topic.c_str());
                return true;
            }

            ROS_WARN("Adding new input topic [%s].", req.topic.c_str());
            input_subs_[req.topic] = nh_.subscribe<MuxedType>(req.topic, 1, boost::bind(&MsgMux<MuxedType>::msgCB, this, _1, req.topic));
        }

        resp.success = true;
        current_topic_ = req.topic;

        ROS_INFO("Switching to topic [%s].", req.topic.c_str());
        statusUpdateCB(); // Publish updates...
        return true;
    } /* MsgMux::setInputService() */

    /**
     * @brief service call to iterate through inputs
     * @param req ros service request
     * @param resp ros service response
     **/
    bool iterateInputService(
        std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& resp
    ) {
        resp.success = true;

        std::string current_topic = current_topic_;

        // We love that the std::map is ordered so this works :)
        // Even though they aren't ordered how they are inserted... but still fine :)
        bool found_topic = false;
        bool iterated_topic = false;
        for (auto it = input_subs_.begin(); it != input_subs_.end(); ++it) {
            // Find the current topic, then next round we set the current_topic to the next one.
            if (found_topic) {
                resp.message = current_topic_ = it->first;
                iterated_topic = true;
                break;
            }
            if(current_topic_ == it->first) {
                found_topic = true;
                continue;
            }
        }
        // If we never iterated topics or are currently not enabled
        // (current_topic_ == "") then we can go to the first topic
        if(!iterated_topic) {
            if(input_subs_.size() == 0) {
                ROS_WARN("Unable to iterate inputs because there are no inputs..");
                resp.success = false;
                resp.message = "";
                return true;
            }
            resp.message = current_topic_ = input_subs_.begin()->first;
        }

        ROS_INFO("Iterated from [%s] to [%s].", current_topic.c_str(), current_topic_.c_str());
        return true;
    }

    /**
     * @brief service call to reverse iterate through inputs
     * @param req ros service request
     * @param resp ros service response
     **/
    bool reverseIterateInputService(
        std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& resp
    ) {
        if(input_subs_.size() == 0) {
            ROS_WARN("Unable to iterate inputs because there are no inputs..");
            resp.success = false;
            resp.message = "";
            return true;
        }

        resp.success = true;
        std::string current_topic = current_topic_;

        // We love that the std::map is ordered so this works :)
        // Even though they aren't ordered how they are inserted... but still fine :)
        std::string last_it_topic = "";
        for (auto it = input_subs_.begin(); it != input_subs_.end(); ++it) {
            // find the current message and set to previous topic;
            if(current_topic_ == it->first) {
                if(last_it_topic == "") {
                    resp.message = current_topic_ = (--input_subs_.end())->first;
                } else {
                    resp.message = current_topic_ = last_it_topic;
                }
                break;
            }
            last_it_topic = it->first;
        }

        ROS_INFO("Iterated from [%s] to [%s].", current_topic.c_str(), current_topic_.c_str());
        return true;
    }

    /**
     * @brief call to publish status updates
     * @param evt ros timer event
     **/
    void statusUpdateCB(const ros::TimerEvent& evt = ros::TimerEvent()) {
        std_msgs::String update_msg;
        update_msg.data = current_topic_;
        status_pub_.publish(update_msg);
    } /* MsgMux::statusUpdateCB() */

private:
    /**
     * @brief inits the node given the above parameters (and nh_)
     * @param params the params to use to init the node
     **/
    void initNode(Params& params) {
        allow_new_topics_ = params.allow_new_topics;

        if (params.output_topic == "") {
            ROS_ERROR("Output topic param is empty. Node will exit now.");
            exit(1); // TODO: there might be better ways to react to this...
        }
        output_topic_ = params.output_topic;
        output_pub_ = nh_.advertise<MuxedType>(params.output_topic, 1, true);
        if (params.status_topic == "") {
            ROS_ERROR("Status topic param is empty. Node will exit now.");
            exit(1);
        }
        status_pub_ = nh_.advertise<std_msgs::String>(params.status_topic, 1, true);

        if (params.set_input_service == "") {
            ROS_ERROR("Set Input Service param is empty. Node will exit now.");
            exit(1);
        }
        set_input_server_ = nh_.advertiseService(params.set_input_service, &MsgMux<MuxedType>::setInputService, this);

        if (params.iterate_input_service != "") {
            iterate_input_server_ = nh_.advertiseService(params.iterate_input_service, &MsgMux<MuxedType>::iterateInputService, this);
        } else {
            ROS_WARN("Iterate Input Service param is empty, service will not be created!");
        }

        if (params.reverse_iterate_input_service != "") {
            reverse_iterate_input_server_ = nh_.advertiseService(params.reverse_iterate_input_service, &MsgMux<MuxedType>::reverseIterateInputService, this);
        } else {
            ROS_WARN("Reverse Iterate Input Service param is empty, service will not be created!");
        }

        update_timer_ = nh_.createTimer(ros::Duration(params.update_period_ms / 1000), &MsgMux<MuxedType>::statusUpdateCB, this);

        bool input_topic_sub_created = false;
        for (std::string& topic : params.input_topics) {
            if (topic == "") {
                ROS_WARN("Empty input topic provided. Refusing to subscribe.");
                continue;
            }
            if (topic == params.output_topic) {
                ROS_WARN("Input topic [%s] is the same as requested output topic. Refusing to subscribe.", topic.c_str());
                continue;
            }
            if (topic == params.default_input) {
                input_topic_sub_created = true;
            }

            ROS_INFO("Adding subscriber for [%s]", topic.c_str());
            input_subs_[topic] = nh_.subscribe<MuxedType>(topic, 1, boost::bind(&MsgMux<MuxedType>::msgCB, this, _1, topic));
        }

        if (params.default_input != "") {
            if (!input_topic_sub_created) {
                ROS_INFO("Adding subscriber for default input topic: [%s].", params.default_input.c_str());
                input_subs_[params.default_input] = nh_.subscribe<MuxedType>(params.default_input, 1, boost::bind(&MsgMux<MuxedType>::msgCB, this, _1, params.default_input));
            }
            current_topic_ = params.default_input;

            ROS_INFO("Enabling default input topic: [%s].", params.default_input.c_str());
        } else {
            current_topic_ = "";

            ROS_WARN("Default input topic not provided. No input topic is currently enabled.");
        }

        ROS_INFO("Mux initialized.");
    } /* MsgMux::initNode() */

    //! Ros Nodehandle 
    ros::NodeHandle& nh_;
    
    //! Map of input subscribers for each input topic
    std::map<std::string, ros::Subscriber> input_subs_;

    //! Publisher of the current status
    ros::Publisher status_pub_;

    //! Publisher for the output twist
    ros::Publisher output_pub_;

    //! Server to be used to switch the input topic
    ros::ServiceServer set_input_server_;

    //! Server to be used to iterate through inputs
    ros::ServiceServer iterate_input_server_;

    //! Server to be used to reverse iterate through inputs
    ros::ServiceServer reverse_iterate_input_server_;

    //! Timer to publish status updates
    ros::Timer update_timer_;

    //! The current topic that should be muxed out
    std::string current_topic_;

    //! The output topic (to make sure we don't self-subscribe)
    std::string output_topic_;

    //! Whether or not to allow new topics when requested via the service call
    bool allow_new_topics_;

}; // MsgMux



}; /* namespace twist_tools */

#endif