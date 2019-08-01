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

{ifdynParameter}
#include <dynamic_reconfigure/server.h>
#include <{packageName}/{nodeName}Config.h>

{endifdynParameter}
// ROS message & services includes
{foralldirectPublisher}
#include <{apply-get_cpp_path}.h>
{endforalldirectPublisher}
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
    {forallserviceServer}
    ros::ServiceServer {name}_;
    {endforallserviceServer}
    // todo confirm it should be always defined, even if not used.
    {apply-capitalized_node_name}Config component_config_;
    {apply-capitalized_node_name}Impl component_implementation_;

    /**
     * @brief object constructor.
     */
    {apply-capitalized_node_name}ROS() : np_("~")
    {
        {ifdynParameter}
        // preparing dynamic reconfigure mechanism
        f = boost::bind(&{apply-capitalized_node_name}ROS::configure_callback, this, _1, _2);
        server.setCallback(f);
        {endifdynParameter}
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
        {ifserviceServer}
        // handling Service servers
        {endifserviceServer}
        {forallserviceServer}
        {name}_ = n_.advertiseService<{type}::Request , {type}::Response>("{name}", boost::bind(&{apply-capitalized_node_name}Impl::callback_{name}, &component_implementation_,_1,_2,component_config_));
        {endforallserviceServer}
        {ifserviceClient}
        // handling Service clients
        {endifserviceClient}
        {forallserviceClient}
        component_implementation_.passthrough.client_{name} = n_.serviceClient<{type}>("{name}");
        {endforallserviceClient}
    }

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
    bool configure()
    {
        return component_implementation_.configure(component_config_);
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

    {apply-capitalized_node_name}ROS node;
    if (!node.configure())
    {
        ROS_FATAL("Could not configure the node");
        ROS_FATAL("Please check configuration parameters");
        return -1;
    }

    ros::spin();

    return 0;
}