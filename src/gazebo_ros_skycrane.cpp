#include <algorithm>
#include <assert.h>

#include "msl_skycrane_gazebo_plugins/gazebo_ros_skycrane.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkycrane);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSkycrane::GazeboRosSkycrane()
{
  ResetWrench(this->wrench_msg_);

  // ofile.open("/home/gastd/example.txt");

  tfNames.push_back("engine00");
  tfNames.push_back("engine01");
  tfNames.push_back("engine10");
  tfNames.push_back("engine11");
  tfNames.push_back("engine20");
  tfNames.push_back("engine21");
  tfNames.push_back("engine30");
  tfNames.push_back("engine31");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSkycrane::~GazeboRosSkycrane()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Reset Wrench
void GazeboRosSkycrane::ResetWrench(geometry_msgs::Wrench wmsg)
{
  wmsg.force.x = 0;
  wmsg.force.y = 0;
  wmsg.force.z = 0;
  wmsg.torque.x = 0;
  wmsg.torque.y = 0;
  wmsg.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSkycrane::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  
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
  ROS_INFO("%s",link_name_.c_str());

  this->link_ = _model->GetLink(link_name_);
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

  this->pub_ = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("WrenchTotal", 1);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<msl_skycrane_description::CmdMotorPowerStamped>(
    this->topic_name_,1,
    boost::bind( &GazeboRosSkycrane::UpdateObjectEngines,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosSkycrane::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosSkycrane::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////

// Calculate the thrust force in the motor's frame, F = f(per[i]), axis y == up
inline tf::Vector3 GazeboRosSkycrane::CalculateThrust(double per)
{
  if(per > 100.0) return tf::Vector3(0.0, 12500.0, 0.0);
  else if(per < 0.0)  return tf::Vector3(0.0, 0.0, 0.0);
  else return tf::Vector3(0.0, 125*per, 0.0);
}

// Update the controller
void GazeboRosSkycrane::UpdateObjectEngines(const msl_skycrane_description::CmdMotorPowerStamped::ConstPtr& _msg)
{
  geometry_msgs::Wrench wrench_aux;
  ResetWrench(wrench_aux);

  motorPower = *_msg;

  std::vector<double> power;
  power.push_back(_msg->power.landing_engine00);
  power.push_back(_msg->power.landing_engine01);
  power.push_back(_msg->power.landing_engine10);
  power.push_back(_msg->power.landing_engine11);
  power.push_back(_msg->power.landing_engine20);
  power.push_back(_msg->power.landing_engine21);
  power.push_back(_msg->power.landing_engine30);
  power.push_back(_msg->power.landing_engine31);

  for (int i = 0; i < 8; ++i)
  {
    // ROS_INFO("Power vector: motor #%d power %.2f %%",i,power[i]);
    // ofile << "Power vector: motor #" << i << " power " << power[i] << " %%" << std::endl;
  }
  // tf::Vector3 aux, aux2;

  for (int i = 0; i < 8; ++i)
  {
    try
    {
      listenerTF.lookupTransform("/base_link", tfNames.at(i), ros::Time(0),transforms[i]);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("%s",tfNames.at(i).c_str());
      // ros::Duration(1.0).sleep();
    }

    // force
    wrench_aux.force.x += transforms[i].getBasis()[0].dot(CalculateThrust( power[i] ));
    wrench_aux.force.y += transforms[i].getBasis()[1].dot(CalculateThrust( power[i] ));
    wrench_aux.force.z += transforms[i].getBasis()[2].dot(CalculateThrust( power[i] ));

    // ROS_INFO  ("Force = (%f; %f; %f)", 
    // (double)wrench_aux.force.x, (double)wrench_aux.force.y, 
    // (double)wrench_aux.force.z);

    // aux += transforms[i].getBasis() * CalculateThrust(power[i]);

    // wrench_aux.force.x += aux.x();
    // wrench_aux.force.y += aux.y();
    // wrench_aux.force.z += aux.z();

    // ROS_INFO  ("aux fo = (%f; %f; %f)\n", 
    // (double)aux.x(), (double)aux.y(), 
    // (double)aux.z());

    // torque
    // Tx = ry x Fz
    wrench_aux.torque.x += (transforms[i].getOrigin().y() - 5) * transforms[i].getBasis()[2].dot(CalculateThrust( power[i] ));
    // Tx = rz x Fy
    wrench_aux.torque.x -= transforms[i].getOrigin().z() * transforms[i].getBasis()[1].dot(CalculateThrust( power[i] ));
    
    // Ty = rx x Fz
    wrench_aux.torque.y += transforms[i].getOrigin().x() * transforms[i].getBasis()[2].dot(CalculateThrust( power[i] ));
    // Ty = rz x Fx
    wrench_aux.torque.y -= transforms[i].getOrigin().z() * transforms[i].getBasis()[0].dot(CalculateThrust( power[i] ));
    
    // Tz = ry x Fx
    wrench_aux.torque.z += (transforms[i].getOrigin().y() - 5) * transforms[i].getBasis()[0].dot(CalculateThrust( power[i] ));
    // Tz = rx x Fy
    wrench_aux.torque.z -= transforms[i].getOrigin().x() * transforms[i].getBasis()[1].dot(CalculateThrust( power[i] ));

    // ROS_INFO  ("Torque = (%f; %f; %f)", 
    //   (double)wrench_aux.torque.x, (double)wrench_aux.torque.y, 
    //   (double)wrench_aux.torque.z);
}

  // ROS_INFO  ("Force = (%f; %f; %f)", 
  //   (double)wrench_aux.force.x, (double)wrench_aux.force.y, 
  //   (double)wrench_aux.force.z);

  // ofile << "Force = (" << (double)wrench_aux.force.x << "; "
  //                      << (double)wrench_aux.force.y << "; "
  //                      << (double)wrench_aux.force.z << ";)"
  //                      << std::endl;
  
  // ROS_INFO  ("Torque = (%f; %f; %f)\n", 
  //   (double)wrench_aux.torque.x, (double)wrench_aux.torque.y, 
  //   (double)wrench_aux.torque.z);

  // ofile << "Torque = (" << (double)wrench_aux.torque.x << "; "
  //                       << (double)wrench_aux.torque.y << "; "
  //                       << (double)wrench_aux.torque.z << ";)"
  //                       << std::endl;

  // ofile << std::endl;

  geometry_msgs::WrenchStamped msg;

  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();
  msg.wrench = wrench_aux;

  this->pub_.publish(msg);

  this->wrench_msg_ = wrench_aux;
}

math::Vector3 GazeboRosSkycrane::UpdateForce()
{
  math::Vector3 newForce;

  newForce.x = 0;
  newForce.y = 0;
  newForce.z = 0;

  std::vector<double> power;
  power.push_back(motorPower.power.landing_engine00);
  power.push_back(motorPower.power.landing_engine01);
  power.push_back(motorPower.power.landing_engine10);
  power.push_back(motorPower.power.landing_engine11);
  power.push_back(motorPower.power.landing_engine20);
  power.push_back(motorPower.power.landing_engine21);
  power.push_back(motorPower.power.landing_engine30);
  power.push_back(motorPower.power.landing_engine31);

  for (int i = 0; i < 8; ++i)
  {
    try
    {
      listenerTF.lookupTransform("/base_link", tfNames.at(i), ros::Time(0),transforms[i]);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("%s",tfNames.at(i).c_str());
      // ros::Duration(1.0).sleep();
    }

    // force
    newForce.x += transforms[i].getBasis()[0].dot(CalculateThrust( power[i] ));
    newForce.y += transforms[i].getBasis()[1].dot(CalculateThrust( power[i] ));
    newForce.z += transforms[i].getBasis()[2].dot(CalculateThrust( power[i] ));

    // ROS_INFO  ("Force = (%f; %f; %f)", 
    // (double)newForce.x, (double)newForce.y, 
    // (double)newForce.z);
  }

  return newForce;
}


math::Vector3 GazeboRosSkycrane::UpdateTorque()
{
  math::Vector3 newTorque;

  newTorque.x = 0;
  newTorque.y = 0;
  newTorque.z = 0;

  std::vector<double> power;
  power.push_back(motorPower.power.landing_engine00);
  power.push_back(motorPower.power.landing_engine01);
  power.push_back(motorPower.power.landing_engine10);
  power.push_back(motorPower.power.landing_engine11);
  power.push_back(motorPower.power.landing_engine20);
  power.push_back(motorPower.power.landing_engine21);
  power.push_back(motorPower.power.landing_engine30);
  power.push_back(motorPower.power.landing_engine31);

  for (int i = 0; i < 8; ++i)
  {
    try
    {
      listenerTF.lookupTransform("/base_link", tfNames.at(i), ros::Time(0),transforms[i]);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("%s",tfNames.at(i).c_str());
      // ros::Duration(1.0).sleep();
    }

    // Tx = rz x Fy
    newTorque.x += transforms[i].getBasis()[2].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().y();
    // Tx = ry x Fz
    newTorque.x -= transforms[i].getBasis()[1].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().z();
    // Ty = rz x Fx 
    newTorque.y += transforms[i].getBasis()[2].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().x();
    // Ty = rx x Fz
    newTorque.y -= transforms[i].getBasis()[0].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().z();
    // Tz = rx x Fy
    newTorque.z += transforms[i].getBasis()[0].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().y();
    // Tz = ry x Fx
    newTorque.z -= transforms[i].getBasis()[1].dot(CalculateThrust( power[i] )) * transforms[i].getOrigin().x();

    // ROS_INFO  ("Torque = (%f; %f; %f)", 
    // (double)newTorque.x, (double)newTorque.y, 
    // (double)newTorque.z);
  }


  return newTorque;

}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSkycrane::UpdateChild()
{
  this->lock_.lock();
  // math::Vector3 force(0,0,0);
  // math::Vector3 force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
  math::Vector3 force = UpdateForce();
  // math::Vector3 torque(0, 0, 0);
  // math::Vector3 torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
  math::Vector3 torque = UpdateTorque();
  this->link_->AddForce(force);
  this->link_->AddTorque(torque);

  geometry_msgs::WrenchStamped msg;

  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();
  msg.wrench.force.x = force.x;
  msg.wrench.force.y = force.y;
  msg.wrench.force.z = force.z;
  msg.wrench.torque.x = torque.x;
  msg.wrench.torque.y = torque.y;
  msg.wrench.torque.z = torque.z;

  this->pub_.publish(msg);

  this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosSkycrane::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
