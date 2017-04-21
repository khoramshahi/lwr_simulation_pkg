

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>




namespace gazebo
{
class GazeboJointStateInterface : public ModelPlugin
{
protected:
	physics::ModelPtr model;


private:
	ros::NodeHandle node_handle;
	ros::Publisher pub_js;

private:
	event::ConnectionPtr update_connection;


public: GazeboJointStateInterface()
{
	ROS_INFO("Creating GazeboJointStateInterface plugin ...");
}

public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	ROS_INFO("Loading GazeboJointStateInterface");

	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}

	std::string armNamespace = _parent->GetName();
	ROS_INFO_STREAM("A model is deteceted: " << armNamespace);



	if (_sdf->HasElement("robot_components_namespace"))
	{
		sdf::ElementPtr armParamElem = _sdf->GetElement("robot_components_namespace");
		armNamespace = armParamElem->Get<std::string>();
		ROS_INFO_STREAM("Using the namespace prefix : " << armNamespace);

	}
	else
	{
		ROS_WARN("SDF Element 'robot_components_namespace' not defined, so using robot name as namespace for components.");
	}

	//gazebo::physics::Joint_V joints;
	gazebo::physics::Joint_V::const_iterator it;
	//    std::vector<std::string> joint_names;


	for (it = _parent->GetJoints().begin(); it != _parent->GetJoints().end(); ++it)
	{
		physics::JointPtr joint = *it;
		std::string _jointName = joint->GetName();

		unsigned int axis = 0;
		if (joint->GetAngleCount() != 1)
		{
			ROS_FATAL("Only support 1 axis");
			exit(1);
		}

		double currAngle = joint->GetAngle(axis).Radian();
		ROS_INFO_STREAM("We have: " << _jointName << " with current angle: " << currAngle);

		//	    	_parent->Remove


	}


	model = _parent;
	pub_js = node_handle.advertise<sensor_msgs::JointState>("/gazebo/joint_state", 1000);

	update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointStateInterface::WorldUpdate, this));



}


void WorldUpdate()
{
	int i = 0;
	sensor_msgs::JointState js;
	readJointStates(js);
	pub_js.publish(js);
}


void readJointStates(sensor_msgs::JointState& js)
{
	js.header.stamp = ros::Time::now();
	js.header.frame_id = "world";

	gazebo::physics::Joint_V::const_iterator it;
	for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
	{
		physics::JointPtr joint = *it;
		std::string _jointName = joint->GetName();


		unsigned int axis = 0;
		if (joint->GetAngleCount() != 1)
		{
			ROS_FATAL("Only support 1 axis");
			exit(1);
		}

		double currAngle = joint->GetAngle(axis).Radian();
		double currEff = joint->GetForce(axis);
		double currVel = joint->GetVelocity(axis);

		// ROS_INFO("Joint %s (%u) %f %f %f", _jointName.c_str(), i, currAngle, currEff, currVel);

		js.name.push_back(_jointName);
		js.position.push_back(currAngle);
		js.velocity.push_back(currVel);
		js.effort.push_back(currEff);
	}

}




};
GZ_REGISTER_MODEL_PLUGIN(GazeboJointStateInterface)
}
