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
#include <octomap_msgs/GetOctomap.h>

#include <visualization_msgs/MarkerArray.h>


#include <diane_octomap/StairInfo.h>
#include <diane_octomap/StairArrayInfo.h>
#include <diane_octomap/DetectStairs.h>


namespace diane_octomap {



/*!
 * \class DianeOctomapNodelet
 * \brief The DianeOctomapNodelet class
 */
class DianeOctomapNodelet : public DianeOctomap, public nodelet::Nodelet
{
     /// ROS node handle.
     ros::NodeHandle nodeHandle;



     ///Declaring the Publishers
     ros::Publisher msgModeledStairPub;

     ros::Publisher msgModeledStairAllPub;



     ///Declaring the Publishers needed for Visualization
     ros::Publisher msgOctomapOccupiedMarkerPub;

     ros::Publisher msgOccupiedBoundingBoxMarkerPub;

     ros::Publisher msgFirstFilteredOccuppiedPointsPub;

     ros::Publisher msgHoughLinesPub;

     ros::Publisher msgFilteredHoughLinesPub;

     ros::Publisher msgSequencedLinesSegmentsPub;

     ros::Publisher msgStairModelPointsPub;

     ros::Publisher msgModeledStairVisualPub;



     ///Declaring the Subscribers
     ros::Subscriber msgResetOctomapServerSub;              //Subscriber expondo um modo para resetar o Octomap Server;

     ros::Subscriber msgStartVisualizationPublishesSub;     //Subscriber expondo uma maneira de publicar as informações de dentro do algorítmo de uma maneira visual;



     ///Declaring the Services
     ros::ServiceServer srvDetectStairsFromFileSer;         //Serviço expondo o método para se requisitar a detecção da escada;
     ros::ServiceServer srvDetectStairsFromServerSer;       //Serviço expondo o método para se requisitar a detecção da escada;


     ///Declaring the Clients
     ros::ServiceClient srvResetOctomapServerCli;           //Client para requisitar o Reset do Octomap Server (de modo a limpar o mapa);

     ros::ServiceClient srvRequestFullOctomapCli;           //Client para requisitar o mapa completo contido no Octomap Server;



 protected:

     ///Publishing Methods
     void PublishStairModel(Stair* Modeled_Stair);

     void PublishAllStairsModel(vector<Stair*> Modeled_Stairs);


     ///Publishing Methods for Visualization
     void PublishOccupiedMarker();

     void PublishOccupiedBoundingBoxMarker();

     void PublishFirstFilteredOccupiedPoints();

     void PublishHoughLines();

     void PublishFilteredHoughLines();

     void PublishSequencedLinesSegments();

     void PublishStairModelPoints();

     void PublishStairModelsVisual(vector<Stair*> Modeled_Stairs);


     ///Message Callback Methods
     void TreatResetOctomapServerCallback(const std_msgs::Bool::ConstPtr &msg);

     void TreatStartVisualizationPublishesCallBack(const std_msgs::Bool::ConstPtr &msg);


     ///Services Callback Methods
     bool TreatDetectStairsFromFileCallback(std_srvs::Empty::Request & req , std_srvs::Empty::Response & res);

     bool TreatDetectStairsFromServerCallback(diane_octomap::DetectStairs::Request &req, diane_octomap::DetectStairs::Response &res);


     ///Auxiliary Methods
     bool ResetOctomapServer();

     bool RequestOctomapServerOctree();


 public:
     DianeOctomapNodelet();
     void onInit();
     virtual ~DianeOctomapNodelet();
};


}
#endif
