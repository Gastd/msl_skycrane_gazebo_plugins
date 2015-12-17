#include "msl_skycrane_gazebo_plugins/skycrane_plugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SkycranePlugin);


SkycranePlugin::SkycranePlugin()
{
}

void SkycranePlugin::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
	// Get the world name.
	this->world_ = _model->GetWorld();
	// gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );
	// Make sure the ROS node for Gazebo has already been initialized
	// gazebo_ros_->isInitialized();
	// load parameters
	this->robot_namespace_ = "";
	if (_sdf->HasElement("robotNamespace"))
	this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

	if (!_sdf->HasElement("bodyName"))
	{
	ROS_FATAL("force plugin missing <bodyName>, cannot proceed");
	return;
	}
	else
	this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

	this->link_ = _model->GetLink(this->link_name_);
	if (!this->link_)
	{
	ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_.c_str());
	return;
	}

	if (!_sdf->HasElement("topicName"))
	{
	ROS_FATAL("force plugin missing <topicName>, cannot proceed");
	return;
	}
	else
	this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
	<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	return;
	}

	this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
}

SkycranePlugin::~SkycranePlugin()
{
}

}
