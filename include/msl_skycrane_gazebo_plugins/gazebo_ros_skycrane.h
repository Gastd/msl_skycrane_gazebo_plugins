#ifndef SKYCRANE_PLUGIN_H
#define SKYCRANE_PLUGIN_H

#include <string>
#include <vector>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

// Ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <msl_skycrane_description/CmdMotorPowerStamped.h>

// Boost
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <fstream>

namespace gazebo
{

class GazeboRosSkycrane : public ModelPlugin
{
public:
    /// \brief Constructor
    GazeboRosSkycrane();

    /// \brief Destructor
    virtual ~GazeboRosSkycrane();

protected:
    // Documentation inherited
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    virtual void UpdateChild();

private:

    /// \brief call back when a Wrench message is published
    /// \param[in] _msg The Incoming ROS message representing the new force to exert.
    void UpdateObjectForce(const geometry_msgs::WrenchStamped::ConstPtr& _msg);
    void UpdateObjectEngines(const msl_skycrane_description::CmdMotorPowerStamped::ConstPtr& _msg);

    void ResetWrench(geometry_msgs::Wrench wmsg);

    /// \brief The custom callback queue thread function.
    void QueueThread();

    /// \brief A pointer to the gazebo world.
    physics::WorldPtr world_;

    /// \brief A pointer to the Link, where force is applied
    physics::LinkPtr link_;

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    ros::NodeHandle* rosnode_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    boost::mutex lock_;

    /// \brief ROS Wrench topic name inputs
    std::string topic_name_;
    /// \brief The Link this plugin is attached to, and will exert forces on.
    std::string link_name_;

    /// \brief for setting ROS name space
    std::string robot_namespace_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    /// \brief Thead object for the running callback Thread.
    boost::thread callback_queue_thread_;
    /// \brief Container for the wrench force that this plugin exerts on the body.
    geometry_msgs::Wrench wrench_msg_;
    geometry_msgs::WrenchStamped wrench_msg_stamped_;

    msl_skycrane_description::CmdMotorPowerStamped motorPower;

    /// \brief Container for listener the transform.
    tf::TransformListener listenerTF;
    // tf::StampedTransform transform;
    /// \brief Transform between base_link frame and engine frame.
    tf::StampedTransform transforms[8];
    std::vector<std::string> tfNames;

    // std::ofstream ofile;

    inline tf::Vector3 CalculateThrust(double per);

    math::Vector3 UpdateForce();
    math::Vector3 UpdateTorque();


    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;
};

}
#endif
