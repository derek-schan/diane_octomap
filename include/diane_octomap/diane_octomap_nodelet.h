/*!
 * \file diane_octomap_nodelet.h
 */



#ifndef DIANE_DIANE_OCTOMAP_NODELET_H
#define DIANE_DIANE_OCTOMAP_NODELET_H

#include <diane_octomap/diane_octomap.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <string>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <visualization_msgs/MarkerArray.h>


namespace diane_octomap {



/*!
 * \class DianeOctomapNodelet
 * \brief The DianeOctomapNodelet class
 */
class DianeOctomapNodelet : public DianeOctomap, public nodelet::Nodelet
{
     /// ROS node handle.
     ros::NodeHandle nodeHandle;

     ros::Publisher msgOctomapFullMapPub;
     ros::Publisher msgOctomapOccupiedMarkerPub;
     ros::Publisher msgOctomapFreeMarkerPub;
     ros::Publisher msgOctomapStair;


     ros::Subscriber msgBoolSub;

     ros::Subscriber msgOctomapFullMapSub;

//     std::vector <ros::Subscriber> msgeposstatesub;
//     std::vector <ros::Subscriber> msgeposinfosub;
//     ros::Subscriber msgKinectPitchSub;

//     ros::Publisher msginputpub;
//     ros::Publisher msgFeedbackPub;
//     std::vector <ros::Publisher> msgEposInfoPub;

//     std::vector <ros::ServiceClient> srveposcontrolcli;

//     void EposStateCallback(const std_msgs::UInt8::ConstPtr& msg, int i);
//     void EposInfoCallback(const epos::Info::ConstPtr& msg, int i);
//     void KinectPitchCallback(const std_msgs::Float64::ConstPtr& msg);
//     void LoadVariablesInEpos(float* pos, short* controlMode, std::vector<bool>* digitalOut);
//     void Feedback();

 protected:
     //Métodos de publicacão
     void PublishOctomapFullMap();
     void publisherGraf(Stair* stair);

     void PublishOccupiedMarker();

     //Métodos de Callback
     void TreatCallBack(const std_msgs::Bool::ConstPtr &msg);

     void TreatOctomapFullMapCallback(const octomap_msgs::Octomap::ConstPtr &msg);

 public:
     DianeOctomapNodelet();
     void onInit();
     virtual ~DianeOctomapNodelet();
};


}
#endif
