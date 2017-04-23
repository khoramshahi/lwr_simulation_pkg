#include <ros/ros.h>
//
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "geometry_msgs/WrenchStamped.h"
//
//
#include <urdf/model.h>
//
//
//#include <tf/transform_listener.h>
//#include <tf_conversions/tf_kdl.h>
//
//#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
//#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
//
//#include <mutex>




class GazeboLwrForwardKinematics
{
public:
	GazeboLwrForwardKinematics()
{

}

	~GazeboLwrForwardKinematics()
	{

	}



	void initialize()
	{
		ROS_INFO("The forward kinematic module is being initialized");

		ros::NodeHandle n("");

		root_name = "lwr_base_link";
		tip_name = "lwr_7_link";

        if (!ros::param::search(n.getNamespace(), "robot_description", robot_description))
            ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");

        std::string xml_string;
        if (n.hasParam(robot_description))
            n.getParam(robot_description.c_str(), xml_string);
        else
        {
            ROS_ERROR("Parameter %s not set, ...", robot_description.c_str());
        }

        if (xml_string.size() == 0)
            ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());

        ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

         if (!model.initString(xml_string))
             ROS_ERROR("Failed to parse urdf file");

         ROS_INFO("Successfully parsed urdf file");

         if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
             ROS_ERROR("Failed to construct kdl tree");

         // Populate the KDL chain
         if(!kdl_tree.getChain(root_name, tip_name, kdl_chain))
         {
             ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
             ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
             ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
             ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
             ROS_ERROR_STREAM("  The segments are:");

             KDL::SegmentMap segment_map = kdl_tree.getSegments();
             KDL::SegmentMap::iterator it;

             for( it=segment_map.begin(); it != segment_map.end(); it++ )
               ROS_ERROR_STREAM( "    "<<(*it).first);
         }

         ROS_INFO("Number of segments: %d", kdl_chain.getNrOfSegments());
         ROS_INFO("Number of joints in chain: %d", kdl_chain.getNrOfJoints());

//        if (!n.getParam("root_name", root_name))
//        {
//            ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
//        }
//
//        if (!n.getParam("tip_name", tip_name))
//        {
//            ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
//        }




	}






private:

//	ros::Subscriber sub_position;
//	ros::Subscriber sub_velocity;
//	ros::Publisher pub_force;
//
//
//	geometry_msgs::WrenchStamped force_spring;
//	geometry_msgs::WrenchStamped force_damping;
//	geometry_msgs::WrenchStamped force_total;
//
//	std::mutex m;
//
//
	std::string robot_namespace, robot_description, root_name, tip_name;
//
//	tf::TransformListener listener;
//	tf::StampedTransform transform;
//
//	KDL::Frame robotBase_kdlFrame;
	KDL::Chain kdl_chain;
	KDL::Tree kdl_tree;
//
//	ros::Publisher pub_cart_pose;
	urdf::Model model;

};//End of class SubscribeAndPublish


// this function closes the application
//void close(void);







// Since flag is

int main(int argc, char **argv)
{

	//setup callback when application exits
	//	atexit(close);


	//Initiate ROS
	ros::init(argc, argv, "GazeboLwrForwardKinematics");

	GazeboLwrForwardKinematics GazeboLwrForwardKinematics_object;

	GazeboLwrForwardKinematics_object.initialize();


	ros::spin();

	return 0;
}

