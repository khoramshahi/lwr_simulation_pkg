#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>




#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/String.h>

#include <string>
//#include <iterator>


namespace gazebo
{
class GazeboJointCommandInterface : public ModelPlugin
{

public:
	physics::WorldPtr world;
	physics::ModelPtr model;

	std::string topic_name;

	ros::NodeHandle node_handle;
	ros::Subscriber sub_jointCommands;


public: GazeboJointCommandInterface()
{
	ROS_INFO("Creating GazeboJointCommandInterface plugin ...");

}

public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

	ROS_INFO("Loading GazeboJointCommandInterface");

    world = _parent->GetWorld();
	ROS_INFO_STREAM("Gazebo world is deteced; Name: " << world->GetName());


	std::string robotName = _parent->GetName();
	ROS_INFO_STREAM("A robot model is deteceted: " << robotName);


//    if ( _sdf->HasElement("robotNamespace") )
//    	this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";



	if (_sdf->HasElement("robot_components_namespace"))
	{
		sdf::ElementPtr armParamElem = _sdf->GetElement("robot_components_namespace");
		std::string armNamespace = armParamElem->Get<std::string>();
		ROS_INFO_STREAM("The robot components namespace is detected : " << armNamespace);
	}
	else
	{
		ROS_WARN("SDF Element 'robot_components_namespace' not defined, so using robot name as namespace for components.");
	}


	// Safety check
	if (_parent->GetJointCount() == 0)
	{
		ROS_ERROR("There is no joint to be loaded\n");
		return;
	}
	ROS_INFO_STREAM("Number of detected joints: " << _parent->GetJointCount());


	// Store the model pointer for convenience.
	model = _parent;




    if ( !_sdf->HasElement("topicName") )
    {
    	topic_name = "/gazebo/joint_commands";
        ROS_WARN("No rostopic name is detected, using the default name : %s", topic_name.c_str() );
    }
    else
    {
    	topic_name = _sdf->GetElement("topicName")->Get<std::string>();
    }

	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}

	sub_jointCommands = node_handle.subscribe( topic_name, 1, &GazeboJointCommandInterface::CallBackMethod, this, ros::TransportHints().tcpNoDelay() );




}

void CallBackMethod(sensor_msgs::JointStateConstPtr  _msg)
{
    // From: https://gist.github.com/alexsleat/1372845

    int i = 0;
    // float64 is implemented as double, see http://wiki.ros.org/msg#Fields
    for (std::vector<double>::const_iterator it = _msg->effort.begin(); it != _msg->effort.end(); ++it )
    {
    	ROS_INFO_STREAM("For joint :  " <<  _msg->name[i] << " received desired fore =  "<< *it);
//        this->fp_torques[i] = *it;
        i++;
    }
}


};
GZ_REGISTER_MODEL_PLUGIN(GazeboJointCommandInterface)
}
