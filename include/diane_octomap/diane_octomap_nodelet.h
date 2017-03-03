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


#include <diane_octomap/StairInfo.h>
#include <diane_octomap/StairArrayInfo.h>


namespace diane_octomap {



/*!
 * \class DianeOctomapNodelet
 * \brief The DianeOctomapNodelet class
 */
class DianeOctomapNodelet : public DianeOctomap, public nodelet::Nodelet
{
     /// ROS node handle.
     ros::NodeHandle nodeHandle;

     //Declarando os Publishers das Mensagens
     ros::Publisher msgOctomapFullMapPub;
     ros::Publisher msgOctomapOccupiedMarkerPub;
     ros::Publisher msgOctomapFreeMarkerPub;

     ros::Publisher msgModeledStairVisualPub;

     ros::Publisher msgModeledStairPub;

     ros::Publisher msgModeledStairAllPub;


     //Declarando os Publishers das Mensagens para o vídeo
     ros::Publisher msgFirstFilteredOccuppiedPointsPub;



     //Declarando os Subscribers de Mensagens
     ros::Subscriber msgBoolSub;

     ros::Subscriber msgOctomapFullMapSub;


     //Declarando os Servers dos Servicos
     ros::ServiceServer srvDetectStairsSer;


     //Declarando os Clients dos Servicos
     //std::vector <ros::ServiceClient> srveposcontrolcli;



 protected:
     //Métodos de Publicacão
     void PublishOctomapFullMap();

     void PublishOccupiedMarker();

     void PublishStairModelsVisual(vector<Stair*> Modeled_Stairs);

     void PublishStairModel(Stair* Modeled_Stair);

     void PublishAllStairsModel(vector<Stair*> Modeled_Stairs);


     //Métodos de Publicacão para o Vídeo
     void PublishFirstFilteredOccupiedPoints();


     //Métodos de Callback
     void TreatBoolCallBack(const std_msgs::Bool::ConstPtr &msg);

     void TreatOctomapFullMapCallback(const octomap_msgs::Octomap::ConstPtr &msg);


     //Método de callback do servico que inicializa a deteccão da escada
     bool DetectStairsCallback(std_srvs::Empty::Request & req , std_srvs::Empty::Response & res);


 public:
     DianeOctomapNodelet();
     void onInit();
     virtual ~DianeOctomapNodelet();
};


}
#endif
