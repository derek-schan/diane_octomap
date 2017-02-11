// diane_octomap_nodelet.cpp



#include <diane_octomap/diane_octomap_nodelet.h>


using octomap_msgs::Octomap;
using namespace std;


diane_octomap::DianeOctomapNodelet::DianeOctomapNodelet()
{
}


void diane_octomap::DianeOctomapNodelet::onInit()
{
    nodeHandle = getNodeHandle();

    //Inicializando os Publishers de Mensagens
    msgOctomapFullMapPub = nodeHandle.advertise <Octomap> (getName() + "/octomap_full", 1000, true);
    msgOctomapOccupiedMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/occupied_cells_vis_array", 1000, true);
    msgOctomapFreeMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/free_cells_vis_array", 1000, true);

    msgModeledStairVisualPub = nodeHandle.advertise<visualization_msgs::Marker>(getName() + "/Modeled_Stairs_Visualization_Markers", 10);

//    msgModeledStairPub = nodeHandle.advertise <std_msgs::Float64MultiArray>("Stair_msg",10);
    msgModeledStairPub = nodeHandle.advertise <diane_octomap::StairInfo>("Stair_msg",10);

    //Inicializando os Subscribers de Mensagens
    msgBoolSub = nodeHandle.subscribe <std_msgs::Bool> ("/bool_msg", 10, &DianeOctomapNodelet::TreatBoolCallBack, this);
    msgOctomapFullMapSub = nodeHandle.subscribe <Octomap> ("/octomap_full", 10, &DianeOctomapNodelet::TreatOctomapFullMapCallback, this);


    //Inicializando os Servers de Servicos
    srvDetectStairsSer = nodeHandle.advertiseService(getName() + "/Detect_Stairs", &DianeOctomapNodelet::DetectStairsCallback, this);


    //Iniciando o Ciclo do Thread
    StartInternalCycle();

}


//Publicando a octree que está armazenada internamente (Caso outro pacote queira utilizar)
void diane_octomap::DianeOctomapNodelet::PublishOctomapFullMap()
{
    Octomap map;
    map.header.frame_id = "/map";
    map.header.stamp = ros::Time::now();


    if (octomap_msgs::fullMapToMsg(*octree, map))
    {
        msgOctomapFullMapPub.publish(map);
    }
    else
    {
        ROS_ERROR("Error serializing OctoMap");
    }
}


//Publicando um MarkerArray com os voxels ocupados
void diane_octomap::DianeOctomapNodelet::PublishOccupiedMarker()
{
    size_t octomapSize = octree->size();
    if (octomapSize <= 1)
    {
        ROS_WARN("Nothing to publish, octree is empty");
        return;
    }

    // init markers of occupied voxels:
    visualization_msgs::MarkerArray occupiedNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(octree->getTreeDepth() + 1);

    // now, traverse all leafs in the tree e completando o OccupiedMarker:
    std_msgs::ColorRGBA _color; _color.r = (1.0); _color.g = (1.0); _color.b = (1.0); _color.a = 1.0;

    for(OcTree::leaf_iterator it = octree->begin(), end = octree->end(); it!= end; ++it)
    {
        //Verificar se o voxel faz parte do Bounding Box "BBX"


        //Se o nó estiver ocupado:
        if (octree->isNodeOccupied(*it))
        {
            double size = it.getSize();
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            //create marker:
            unsigned idx = it.getDepth();
            assert(idx < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

            //Definindo a cor (Branco somente para visualizacão)
            occupiedNodesVis.markers[idx].colors.push_back(_color);

        }

    }

    ros::Time rostime = ros::Time::now();

    //Completando as informacões de cada Marker presente no Occupied Marker Array
    for(unsigned i=0; i < occupiedNodesVis.markers.size(); ++i)
    {
        //Obtendo a quantidade de voxels na octree na profundidade indicada
        double size = octree->getNodeSize(i);

        occupiedNodesVis.markers[i].header.frame_id = "/map";
        occupiedNodesVis.markers[i].header.stamp = rostime;
        occupiedNodesVis.markers[i].ns = "map";
        occupiedNodesVis.markers[i].id = i;
        occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].scale.x = size;
        occupiedNodesVis.markers[i].scale.y = size;
        occupiedNodesVis.markers[i].scale.z = size;

        if(occupiedNodesVis.markers[i].points.size() > 0)
        {
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

    }

    msgOctomapOccupiedMarkerPub.publish(occupiedNodesVis);

}


//Publicando os Markers que definem os Modelos de Escada detectados
void diane_octomap::DianeOctomapNodelet::PublishStairModelsVisual(vector<diane_octomap::Stair*> Modeled_Stairs)
{

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;


    line_list.id = 0;


    line_list.type = visualization_msgs::Marker::LINE_LIST;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.01;




    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    for(int j = 0; j < Modeled_Stairs.size() ; ++j)
    {
        for (int i = 0; i < Modeled_Stairs.at(j)->Points.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = Modeled_Stairs.at(j)->Points.at(i).at(0);
            p.y = Modeled_Stairs.at(j)->Points.at(i).at(1);
            p.z = Modeled_Stairs.at(j)->Points.at(i).at(2);
            line_list.points.push_back(p);

        }

        for (int i = 0; 2*i < Modeled_Stairs.at(j)->Points.size()-2; ++i)
        {

            geometry_msgs::Point p;
            p.x = Modeled_Stairs.at(j)->Points.at(2*i).at(0);
            p.y = Modeled_Stairs.at(j)->Points.at(2*i).at(1);
            p.z = Modeled_Stairs.at(j)->Points.at(2*i).at(2);
            line_list.points.push_back(p);


        }
        for (int i = 0; 2*i < Modeled_Stairs.at(j)->Points.size() -2; ++i)
        {

            geometry_msgs::Point p;
            p.x = Modeled_Stairs.at(j)->Points.at(2*i+1).at(0);
            p.y = Modeled_Stairs.at(j)->Points.at(2*i+1).at(1);
            p.z = Modeled_Stairs.at(j)->Points.at(2*i+1).at(2);
            line_list.points.push_back(p);


        }



        for (int i = 0; 2*i   < Modeled_Stairs.at(j)->Points.size()-3; ++i)
        {

            geometry_msgs::Point p;
            p.x = Modeled_Stairs.at(j)->Points.at(2*i+3).at(0);
            p.y = Modeled_Stairs.at(j)->Points.at(2*i+3).at(1);
            p.z = Modeled_Stairs.at(j)->Points.at(2*i+3).at(2);
            line_list.points.push_back(p);


        }

        for (int i = 0; 2*i  < Modeled_Stairs.at(j)->Points.size() - 2; ++i)
        {

            geometry_msgs::Point p;
            p.x = Modeled_Stairs.at(j)->Points.at(2*i+2).at(0);
            p.y = Modeled_Stairs.at(j)->Points.at(2*i+2).at(1);
            p.z = Modeled_Stairs.at(j)->Points.at(2*i+2).at(2);
            line_list.points.push_back(p);


        }

    }

    msgModeledStairVisualPub.publish(line_list);

}


void diane_octomap::DianeOctomapNodelet::PublishStairModel(Stair *Modeled_Stair)
{
    diane_octomap::StairInfo msg;

    //Preenchendo as informacões da escada na mensagem a ser enviada
    msg.Total_Length = Modeled_Stair->Total_Length;
    msg.Total_Width = Modeled_Stair->Total_Width;
    msg.Total_Height = Modeled_Stair->Total_Height;

    msg.Min_Z = Modeled_Stair->Min_Z;
    msg.Max_Z = Modeled_Stair->Max_Z;

    msg.Num_Steps = Modeled_Stair->Num_Steps;

    msg.Step_Length = Modeled_Stair->Step_Length;
    msg.Step_Width = Modeled_Stair->Step_Width;
    msg.Step_Height = Modeled_Stair->Step_Height;

    msg.Plane_Alpha = Modeled_Stair->Plane_Alpha;


    msgModeledStairPub.publish(msg);

}



void diane_octomap::DianeOctomapNodelet::TreatBoolCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    PublishOccupiedMarker();

    PublishStairModelsVisual(Modeled_Stairs);


    PublishStairModel(Modeled_Stairs.at(0));

}


void diane_octomap::DianeOctomapNodelet::TreatOctomapFullMapCallback(const Octomap::ConstPtr& msg)
{
    //Será chamado quando recebermos uma mensagem do octomap_server como o mapa completo
    //Lendo o octomap à partir da mensagem recebida (que está sendo publicada pelo Octomap_Server
    AbstractOcTree* abs_tree = octomap_msgs::msgToMap(*msg);

    //Atualizar ou inicializar a octree
    if(octreeFromMsg == NULL)
    {
        octreeFromMsg = dynamic_cast<OcTree*>(abs_tree);
    }
    else
    {
        //Atualiza o mapa de algum jeito
    }

}



bool diane_octomap::DianeOctomapNodelet::DetectStairsCallback(std_srvs::Empty::Request & req , std_srvs::Empty::Response & res)
{
    //Se recebeu uma requisicao para detectar a escada, inicializa as funcoes de deteccao da escada.
    //Após detectar a escada, a mesma continuará sendo publicada por um publisher em uma mensagem

    //Obtendo a octree à partir do arquivo (o caminho para o arquivo ainda está definido chapado no código - mudar para um arquivo de configuracão).
    DianeOctomap::GenerateOcTreeFromFile();

    //Filtrando e armazenando as folhas da octree que estejam dentro da Bounding Box definida no método e que estejam ocupadas.
    DianeOctomap::GetOccupiedLeafsOfBBX(octree);

    //Utilizando as folhas filtradas (presentes no vetor) para detectar as informacões da escada.
    //DianeOctomap::StairDetection();
    DianeOctomap::StairDetection2D();


    return true;

}


diane_octomap::DianeOctomapNodelet::~DianeOctomapNodelet()
{
    StopInternalCycle();
}
