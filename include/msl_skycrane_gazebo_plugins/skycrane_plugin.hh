#ifndef GAZEBO_ROS_SKYCRANE_PLUGIN_HH
#define GAZEBO_ROS_SKYCRANE_PLUGIN_HH

#include <ros/ros.h>
#include "gazebo_plugins/gazebo_ros_template.h"

namespace gazebo
{
	class SkycranePlugin : public GazeboRosTemplate
	{
	public:
		SkycranePlugin();
		~SkycranePlugin();

	/// \brief Load the controller
	void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

	private: physics::WorldPtr world_;
	private: physics::LinkPtr link_;
	private: ros::NodeHandle* rosnode_;
	private: ros::Subscriber sub_;
	private: std::string topic_name_;
	private: std::string link_name_;
	private: std::string robot_namespace_;


	/// \brief Update the controller
	protected: virtual void UpdateChild();

	};
}
#endif
