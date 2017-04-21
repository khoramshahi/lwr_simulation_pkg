

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <map>
#include <string>


namespace gazebo
{
class GazeboJointCommandInterface : public ModelPlugin
{


private:
//	event::ConnectionPtr update_connection;

public:
	physics::ModelPtr model;
//	physics::Link_V updatedLinks;
//	std::map<std::string, physics::JointPtr> joints;
//
//
//
	ros::NodeHandle* node;
	ros::Subscriber sub_jointCommand;
//
//	common::Time prevUpdateTime;
//
//
//	std::map<std::string, double> forces;
//
//
//
//	/// To be implemented late
//	std::map<std::string, double> positions;
//	std::map<std::string, double> velocities;
//
//	std::map<std::string, common::PID> posPids;
//	std::map<std::string, common::PID> velPids;


public: GazeboJointCommandInterface()
{
	ROS_INFO("Creating GazeboJointCommandInterface plugin ...");

}

public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

	ROS_INFO("Loading GazeboJointCommandInterface");


	// Safety check
	if (_parent->GetJointCount() == 0)
	{
		std::cerr << "There is no joint to be loaded\n";
		return;
	}

	std::string armNamespace = _parent->GetName();
	ROS_INFO_STREAM("A model is deteceted: " << armNamespace);

	// Store the model pointer for convenience.
	this->model = _parent;


	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}



	// Create the node
//	this->node = transport::NodePtr(new transport::Node());
//	this->node->Init("my_gazebo_node");

//#if GAZEBO_MAJOR_VERSION < 8
//	ROS_INFO_STREAM("The name of the model is : " << this->model->GetWorld()->GetName());
//	this->node->Init(this->model->GetWorld()->GetName());
//#else
//	ROS_INFO_STREAM("The name of the model is : " << this->model->GetWorld()->Name());
//	this->node->Init(this->model->GetWorld()->Name());
//#endif




    this->node = new ros::NodeHandle("my_gzplugin");


	// Create a topic name
	std::string topicName = "/gazebo/joint_commands";

	// Subscribe to the topic, and register a callback
//	this->sub_jointCommand = this->node->subscribe(topicName, 1, &GazeboJointCommandInterface::OnJointCommand, this, ros::TransportHints().tcpNoDelay());



//	ROS_INFO_STREAM("A Node is inititaded, TopicNameSpace : " << this->node->get getNamespace() ) <<
//			" id : " << this->node->getId() <<
//			" message type : " << this->node-> );



	//std::cout << this->node->GetTopicNamespace() << std::endl;
	//
	//	//	this->joints = model->GetJoints()[3];
	//
	//



}
void OnJointCommand(ConstJointCmdPtr &_msg)
{
	int i =0;
}



};
GZ_REGISTER_MODEL_PLUGIN(GazeboJointCommandInterface)
}
