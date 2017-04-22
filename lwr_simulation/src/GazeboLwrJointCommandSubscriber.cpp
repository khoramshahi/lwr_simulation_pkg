#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>




#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/String.h>
#include <ros/callback_queue.h>


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

	ros::CallbackQueue queue;
	boost::thread callback_queue_thread;

	// this is where we can stor desired position, velocity, or force
	sensor_msgs::JointState desired_js;

private:
	event::ConnectionPtr update_connection;
	boost::mutex mlock;

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
	ROS_INFO_STREAM("Number of joints detected: " << _parent->GetJointCount());

	// create a template for receiving the joint commands
	desired_js.header.stamp = ros::Time::now();
	desired_js.header.frame_id = "world";

	gazebo::physics::Joint_V::const_iterator it;
	for (it = _parent->GetJoints().begin(); it != _parent->GetJoints().end(); ++it)
	{
		physics::JointPtr joint = *it;

		desired_js.name.push_back(joint->GetName());
		desired_js.position.push_back(0);
		desired_js.velocity.push_back(0);
		desired_js.effort.push_back(0);
	}






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

	// setting up the subscriber, callback will be called back!
	sub_jointCommands = node_handle.subscribe( topic_name, 1, &GazeboJointCommandInterface::OnJointCommand, this, ros::TransportHints().tcpNoDelay() );


	// Custom Callback Queue
	callback_queue_thread = boost::thread( boost::bind(&GazeboJointCommandInterface::QueueThread, this) );

	// New Mechanism for Updating every World Cycle
	//Listen to the update event. This event is broadcast every simulation iteration.
	update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointCommandInterface::WorldUpdate, this) );


}

void OnJointCommand(sensor_msgs::JointStateConstPtr  _msg)
{
	// From: https://gist.github.com/alexsleat/1372845

	int i = 0;
	// float64 is implemented as double, see http://wiki.ros.org/msg#Fields
	for (std::vector<double>::const_iterator it_rec = _msg->effort.begin(); it_rec != _msg->effort.end(); ++it_rec )
	{
		int j = 0;
		// float64 is implemented as double, see http://wiki.ros.org/msg#Fields
		for (std::vector<double>::const_iterator it_app = desired_js.effort.begin(); it_app != desired_js.effort.end(); ++it_app )
		{
			if(_msg->name[i] == desired_js.name[j])
			{
				desired_js.effort[j] = _msg->effort[i];
				//ROS_INFO_STREAM("For joint :  " <<  desired_js.name[j] << " received desired force=  "<< *it_rec);

			}
			j++;
		}
		i++;
	}

	return;
}

void QueueThread()
{
	static const double timeout = 0.01;

	while ( node_handle.ok() )
	{
		queue.callAvailable( ros::WallDuration(timeout) );
	}

	return;
}

void WorldUpdate()
{
	mlock.lock();

	int nJoint = 0;
	gazebo::physics::Joint_V::const_iterator it;
	for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
	{
		physics::JointPtr joint = *it;
		joint->SetForce(0,desired_js.effort[nJoint]);

		if (desired_js.effort[nJoint] != 0)
			ROS_INFO_STREAM("For joint number :" << nJoint <<
					" I am applying a foce here of " << desired_js.effort[nJoint]
				             << " from " << desired_js.name[nJoint]<< " to " << joint->GetName());

		nJoint ++;

	}


	mlock.unlock();



}




};
GZ_REGISTER_MODEL_PLUGIN(GazeboJointCommandInterface)
}
