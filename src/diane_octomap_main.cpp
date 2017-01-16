// diane_mapper_main.cpp


#include <ros/ros.h>
#include <nodelet/loader.h>



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "diane_octomap");

    std::vector<std::string> non_ros_args;
    ros::removeROSArgs(argc, argv, non_ros_args);

    nodelet::Loader n(false);
    ros::M_string remappings; //Remappings are already applied by ROS no need to generate them.
    std::string nodelet_name = ros::this_node::getName();
    std::string nodelet_type = "diane_octomap/DianeOctomapNodelet";
    if(!n.load(nodelet_name, nodelet_type, remappings, non_ros_args))
      return -1;

    ros::spin();
}


