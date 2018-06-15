/**
* @file {nodeName}_ros.cpp
* @author {packageAuthor}
*
* Copyright (C) {packageCopyright}
*
* @brief {packageDescription}
* @warning This file should not be edited
**/

// ROS includes
#include <ros/ros.h>

{ifactionServer}
#include <actionlib/server/simple_action_server.h>
{forallactionServer}
#include <{apply-get_cpp_path}Action.h>
{endforallactionServer}
{endifactionServer}
{ifactionClient}
#include <actionlib/client/simple_action_client.h>
{forallactionClient}
#include <{apply-get_cpp_path}Action.h>
{endforallactionClient}
{endifactionClient}
{ifdynParameter}
#include <dynamic_reconfigure/server.h>
#include <{packageName}/{nodeName}Config.h>

{endifdynParameter}
// ROS message & services includes
{forallpublisher}
#include <{apply-get_cpp_path}.h>
{endforallpublisher}
{foralldirectPublisher}
#include <{apply-get_cpp_path}.h>
{endforalldirectPublisher}
{forallsubscriber}
#include <{apply-get_cpp_path}.h>
{endforallsubscriber}
{foralldirectSubscriber}
#include <{apply-get_cpp_path}.h>
{endforalldirectSubscriber}
{forallserviceServer}
#include <{apply-get_cpp_path}.h>
{endforallserviceServer}
{forallserviceClient}
#include <{apply-get_cpp_path}.h>
{endforallserviceClient}

// other includes
#include <{nodeName}_common.cpp>

/**
 * @class {apply-capitalized_node_name}ROS
 * @brief Class handling the connection with the ROS world.
 * It also implement the node life-cycle, and access to object {nodeName}-impl when appropriate
 */
class {apply-capitalized_node_name}ROS
{
public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    {ifdynParameter}
    dynamic_reconfigure::Server<{packageName}::{nodeName}Config> server;
    dynamic_reconfigure::Server<{packageName}::{nodeName}Config>::CallbackType f;
    {endifdynParameter}
    {forallpublisher}
    ros::Publisher {name}_;
    {endforallpublisher}
    {forallsubscriber}
    ros::Subscriber {name}_;
    {endforallsubscriber}
    {forallserviceServer}
    ros::ServiceServer {name}_;
    {endforallserviceServer}
    {forallactionServer}
    actionlib::SimpleActionServer<{type}Action> as_{name}_;
    {endforallactionServer}

    {apply-capitalized_node_name}Data component_data_;
    // todo confirm it should be always defined, even if not used.
    {apply-capitalized_node_name}Config component_config_;
    {apply-capitalized_node_name}Impl component_implementation_;

    /**
     * @brief object constructor.
     */
    {apply-capitalized_node_name}ROS() : np_("~")
                     {forallactionServer}
                     , as_{name}_(n_, "{name}", boost::bind(&{apply-capitalized_node_name}Impl::callback_{name}, &component_implementation_, _1, &as_{name}_), false)
                     {endforallactionServer}
    {
        {ifactionServer}
        // launching action servers
        {endifactionServer}
        {forallactionServer}
        as_{name}_.start();
        {endforallactionServer}
        {ifdynParameter}
        // preparing dynamic reconfigure mechanism
        f = boost::bind(&{apply-capitalized_node_name}ROS::configure_callback, this, _1, _2);
        server.setCallback(f);
        {endifdynParameter}
        {forallpublisher}
        {name}_ = n_.advertise<{type}>("{name}", 1);
        {endforallpublisher}
        {ifdirectPublisher}
        // Handling direct publisher
        {foralldirectPublisher}
        component_implementation_.passthrough.{name} = n_.advertise<{type}>("{name}", 1);
        {endforalldirectPublisher}
        {endifdirectPublisher}
        {ifdirectSubscriber}
        // Handling direct subscriber
        {foralldirectSubscriber}
        component_implementation_.passthrough.{name} = n_.subscribe("{name}", 1, &{apply-capitalized_node_name}Impl::directTopicCallback_{name}, &component_implementation_);
        {endforalldirectSubscriber}
        {endifdirectSubscriber}
        {forallsubscriber}
        {name}_ = n_.subscribe("{name}", 1, &{apply-capitalized_node_name}ROS::topicCallback_{name}, this);
        {endforallsubscriber}
        {ifparameter}
        // handling parameters from the parameter server
        {endifparameter}
        {forallparameter}
        np_.param("{name}", component_config_.{name}, ({type}){apply-get_cpp_param_value});
        {endforallparameter}
        {ifdynParameter}
        // handling dynamic parameters
        {endifdynParameter}
        {foralldynParameter}
        np_.param("{name}", component_config_.{name}, ({type}){apply-get_cpp_param_value});
        {endforalldynParameter}
        {forallserviceServer}
        // to enable service name adjustment when loading the node
        std::string {name}_remap;
        np_.param("{name}_remap", {name}_remap, (std::string)"{name}");
        ROS_INFO_STREAM("Service server at direction " << {name}_remap);
        {name}_ = n_.advertiseService<{type}::Request , {type}::Response>({name}_remap, boost::bind(&{apply-capitalized_node_name}Impl::callback_{name}, &component_implementation_,_1,_2,component_config_));
        {endforallserviceServer}
        {forallserviceClient}
        std::string sc_{name}_remap;
        np_.param("{name}_remap", sc_{name}_remap, (std::string)"{name}");
        ROS_INFO_STREAM("Service client looking at direction " << sc_{name}_remap);
        component_implementation_.passthrough.client_{name} = n_.serviceClient<{type}>(sc_{name}_remap);
        {endforallserviceClient}
        {forallactionClient}
        // to enable action name adjustment when loading the node
        std::string ac_{name}_remap;
        np_.param("{name}_remap", ac_{name}_remap, (std::string)"{name}");
        component_implementation_.passthrough.ac_{name} = new actionlib::SimpleActionClient<{type}Action> (ac_{name}_remap, true);
        ROS_INFO_STREAM("Waiting for action server " << ac_{name}_remap << " to start.");
        // wait for the action server to start
        // will wait for infinite time
        component_implementation_.passthrough.ac_{name}->waitForServer();
        ROS_INFO_STREAM("Action server " << ac_{name}_remap << " started.");
        {endforallactionClient}
    }

    {forallsubscriber}
    /**
     * @brief callback of a topic subscription handled through the update mechanism.
     * @param msg message received from ROS world
     */
    void topicCallback_{name}(const {type}::ConstPtr& msg)
    {
        component_data_.in_{name} = *msg;
        component_data_.in_{name}_updated = true;
    }

    {endforallsubscriber}
    {ifdynParameter}
    /**
     * @brief callback called when a dynamically reconfigurable parameter is changed
     * @param config the parameter structure to be updated
     * @param level level of parameters changed
     */
    void configure_callback({packageName}::{nodeName}Config &config, uint32_t level)
    {
        {foralldynParameter}
        component_config_.{name} = config.{name};
        {endforalldynParameter}
    }

    {endifdynParameter}
    /**
     * @brief configure function called after node creation.
     */
    void configure()
    {
        component_implementation_.configure(component_config_);
    }
    /**
     * @brief Activate all publishers handled through the update mechanism
     */
    void activate_all_output()
    {
        {forallpublisher}
        component_data_.out_{name}_active = true;
        {endforallpublisher}
    }
    /**
     * @brief State that all input has been read
     */
    void all_input_read()
    {
        {forallsubscriber}
        component_data_.in_{name}_updated = false;
        {endforallsubscriber}
     }
    /**
     * @brief core function periodically called.
     * calls implementation update, and handles potential publications
     * @param event access to the timer used for the looping
     */
    void update(const ros::TimerEvent& event)
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        all_input_read();
        {forallpublisher}
        if (component_data_.out_{name}_active)
            {name}_.publish(component_data_.out_{name});
        {endforallpublisher}
    }
    /**
     * @brief object destructor
     */
    ~{apply-capitalized_node_name}ROS()
    {
    }
};

/**
 * @brief Main of the component. Create the object and launches the periodical call to upde method.
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "{nodeName}");

    ros::AsyncSpinner spinner(1);

    {apply-capitalized_node_name}ROS node;
    node.configure();

    ros::Timer timer = node.n_.createTimer(ros::Duration(1.0 / {nodeFrequency}), &{apply-capitalized_node_name}ROS::update, &node);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}