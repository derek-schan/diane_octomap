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
    msgOctomapFullMapPub = nodeHandle.advertise <Octomap> (getName() + "/octomap_full", 10, true);
    msgOctomapOccupiedMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/occupied_cells_vis_array", 10, true);
    msgOctomapFreeMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/free_cells_vis_array", 10, true);

    msgModeledStairVisualPub = nodeHandle.advertise<visualization_msgs::Marker>(getName() + "/Modeled_Stairs_Visualization_Markers", 10);

    msgModeledStairPub = nodeHandle.advertise <diane_octomap::StairInfo>(getName() + "/Modeled_Stair_Info", 10);

    msgModeledStairAllPub = nodeHandle.advertise <diane_octomap::StairArrayInfo>(getName() + "/Modeled_Stairs_Info_All", 10);


    //Publishers de Mensagens para o vídeo
    msgFirstFilteredOccuppiedPointsPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/First_Filtered_Points", 10, true);

    msgHoughLinesPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Hough_Lines", 1000, true);

    msgFilteredHoughLinesPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Filtered_Hough_Lines", 1000, true);

    msgSequencedLinesSegmentsPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Sequenced_Lines_Segments", 1000, true);

    msgStairModelPointsPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/Stair_Model_Points", 10, true);


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

    if(Modeled_Stairs.size() > 0)
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
//        for(int j = 0; j < Modeled_Stairs.size() ; ++j)
        for(int j = 1; j < 2 ; ++j)
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



    else
    {
        ROS_WARN("Nothing to publish! No stairs detected on the map.");
    }


}


void diane_octomap::DianeOctomapNodelet::PublishStairModel(Stair* Modeled_Stair)
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


    vector<float> Aresta;

    for(int i=0; i<Modeled_Stair->Aresta.size(); i++)
    {
        for(int j=0; j<Modeled_Stair->Aresta.at(i).size(); j++)
        {
            Aresta.push_back(Modeled_Stair->Aresta.at(i).at(j));
        }
    }


    msg.Edge_Coordinates = Aresta;



    vector<float> Pontos;

    for(int k=0; k<Modeled_Stair->Points.size(); k++)
    {
        for(int l=0; l<Modeled_Stair->Points.at(k).size(); l++)
        {
            Pontos.push_back(Modeled_Stair->Points.at(k).at(l));
        }
    }

    msg.Points_Coordinates = Pontos;


    msgModeledStairPub.publish(msg);

}


void diane_octomap::DianeOctomapNodelet::PublishAllStairsModel(vector<Stair*> Modeled_Stairs)
{
    diane_octomap::StairArrayInfo msg;


    //Preenchendo as informacões de cada escada modelada na mensagem a ser enviada
    for(int i=0; i<Modeled_Stairs.size(); i++)
    {
        Stair* Modeled_Stair = Modeled_Stairs.at(i);

        diane_octomap::StairInfo stair_info;

        stair_info.Total_Length = Modeled_Stair->Total_Length;
        stair_info.Total_Width = Modeled_Stair->Total_Width;
        stair_info.Total_Height = Modeled_Stair->Total_Height;

        stair_info.Min_Z = Modeled_Stair->Min_Z;
        stair_info.Max_Z = Modeled_Stair->Max_Z;

        stair_info.Num_Steps = Modeled_Stair->Num_Steps;

        stair_info.Step_Length = Modeled_Stair->Step_Length;
        stair_info.Step_Width = Modeled_Stair->Step_Width;
        stair_info.Step_Height = Modeled_Stair->Step_Height;

        stair_info.Plane_Alpha = Modeled_Stair->Plane_Alpha;


        vector<float> Aresta;

        for(int j=0; j<Modeled_Stair->Aresta.size(); j++)
        {
            for(int k=0; k<Modeled_Stair->Aresta.at(j).size(); k++)
            {
                Aresta.push_back(Modeled_Stair->Aresta.at(j).at(k));
            }
        }


        stair_info.Edge_Coordinates = Aresta;



        vector<float> Pontos;

        for(int l=0; l<Modeled_Stair->Points.size(); l++)
        {
            for(int m=0; m<Modeled_Stair->Points.at(l).size(); m++)
            {
                Pontos.push_back(Modeled_Stair->Points.at(l).at(m));
            }
        }

        stair_info.Points_Coordinates = Pontos;


        //Incluindo a escada na mensagem
        msg.Stairs.push_back(stair_info);

    }

    msgModeledStairAllPub.publish(msg);

}



//Publicando um MarkerArray com os voxels ocupados
void diane_octomap::DianeOctomapNodelet::PublishFirstFilteredOccupiedPoints()
{
    size_t FilteredSize = First_Filtered_Points.cols();
    if (FilteredSize <= 1)
    {
        ROS_WARN("Nothing to publish! First filtered occupied points array is empty!");
        return;
    }

    // init markers of occupied voxels:
    visualization_msgs::MarkerArray FirstFilterOccupiedNodesVis;

    //Only has one array:
    FirstFilterOccupiedNodesVis.markers.resize(1);

    //Configuring the color
    std_msgs::ColorRGBA _color; _color.r = (0.0); _color.g = (0.0); _color.b = (1.0); _color.a = 1.0;

    //Completing the FirstFilterOccupiedNodesVis MarkerArray
    for(int i=0; i<First_Filtered_Points.cols(); ++i)
    {
        double x = First_Filtered_Points(0, i);
        double y = First_Filtered_Points(1, i);
        double z = First_Filtered_Points(2, i);

        //create marker:
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        FirstFilterOccupiedNodesVis.markers[0].points.push_back(cubeCenter);

        //Definindo a cor (Branco somente para visualizacão)
        FirstFilterOccupiedNodesVis.markers[0].colors.push_back(_color);
    }

    ros::Time rostime = ros::Time::now();

    for(unsigned int j=0; j < FirstFilterOccupiedNodesVis.markers.size(); ++j)
    {
        //Obtendo a quantidade de voxels na octree na profundidade indicada
        double size = First_Filtered_Points.cols();

        FirstFilterOccupiedNodesVis.markers[j].header.frame_id = "/map";
        FirstFilterOccupiedNodesVis.markers[j].header.stamp = rostime;
        FirstFilterOccupiedNodesVis.markers[j].ns = "map";
        FirstFilterOccupiedNodesVis.markers[j].id = j;
        FirstFilterOccupiedNodesVis.markers[j].type = visualization_msgs::Marker::CUBE_LIST;
        FirstFilterOccupiedNodesVis.markers[j].scale.x = 0.05;
        FirstFilterOccupiedNodesVis.markers[j].scale.y = 0.05;
        FirstFilterOccupiedNodesVis.markers[j].scale.z = 0.05;

        if(FirstFilterOccupiedNodesVis.markers[j].points.size() > 0)
        {
            FirstFilterOccupiedNodesVis.markers[j].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            FirstFilterOccupiedNodesVis.markers[j].action = visualization_msgs::Marker::DELETE;
        }

    }


    msgFirstFilteredOccuppiedPointsPub.publish(FirstFilterOccupiedNodesVis);

}


void diane_octomap::DianeOctomapNodelet::PublishHoughLines()
{
    if(HoughLinesPoints.size() > 0)
    {
        // init markers of occupied voxels:
        visualization_msgs::Marker HoughLinesMarker;

        HoughLinesMarker.header.frame_id = "/map";
        HoughLinesMarker.header.stamp = ros::Time::now();
        HoughLinesMarker.ns = "map";
        HoughLinesMarker.action = visualization_msgs::Marker::ADD;
        HoughLinesMarker.pose.orientation.w = 1.0;

        HoughLinesMarker.id = 15;

        HoughLinesMarker.type = visualization_msgs::Marker::LINE_LIST;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        HoughLinesMarker.scale.x = 0.01;

        HoughLinesMarker.color.g = 1.0;
        HoughLinesMarker.color.a = 1.0;


        for(int i=0; i<HoughLinesPoints.size(); i++)
        {
            geometry_msgs::Point line_point_init;
            line_point_init.x = HoughLinesPoints.at(i)(0,0);
            line_point_init.y = HoughLinesPoints.at(i)(1,0);
            line_point_init.z = HoughLinesPoints.at(i)(2,0);

            geometry_msgs::Point line_point_end;
            line_point_end.x = HoughLinesPoints.at(i)(0,1);
            line_point_end.y = HoughLinesPoints.at(i)(1,1);
            line_point_end.z = HoughLinesPoints.at(i)(2,1);


            HoughLinesMarker.points.push_back(line_point_init);
            HoughLinesMarker.points.push_back(line_point_end);

        }

        msgHoughLinesPub.publish(HoughLinesMarker);


    }
    else
    {
        ROS_WARN("Nothing to publish! No lines were detected!");
        return;
    }

}


void diane_octomap::DianeOctomapNodelet::PublishFilteredHoughLines()
{
    if(FilteredHoughLinesPoints.size() > 0)
    {
        // init markers of occupied voxels:
        visualization_msgs::Marker FilteredHoughLinesMarker;

        FilteredHoughLinesMarker.header.frame_id = "/map";
        FilteredHoughLinesMarker.header.stamp = ros::Time::now();
        FilteredHoughLinesMarker.ns = "map";
        FilteredHoughLinesMarker.action = visualization_msgs::Marker::ADD;
        FilteredHoughLinesMarker.pose.orientation.w = 1.0;

        FilteredHoughLinesMarker.id = 20;

        FilteredHoughLinesMarker.type = visualization_msgs::Marker::LINE_LIST;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        FilteredHoughLinesMarker.scale.x = 0.01;

        FilteredHoughLinesMarker.color.r = 1.0;
        FilteredHoughLinesMarker.color.a = 1.0;


        for(int i=0; i<FilteredHoughLinesPoints.size(); i++)
        {
            geometry_msgs::Point line_point_init;
            line_point_init.x = FilteredHoughLinesPoints.at(i)(0,0);
            line_point_init.y = FilteredHoughLinesPoints.at(i)(1,0);
            line_point_init.z = FilteredHoughLinesPoints.at(i)(2,0);

            geometry_msgs::Point line_point_end;
            line_point_end.x = FilteredHoughLinesPoints.at(i)(0,1);
            line_point_end.y = FilteredHoughLinesPoints.at(i)(1,1);
            line_point_end.z = FilteredHoughLinesPoints.at(i)(2,1);


            FilteredHoughLinesMarker.points.push_back(line_point_init);
            FilteredHoughLinesMarker.points.push_back(line_point_end);

        }

        msgFilteredHoughLinesPub.publish(FilteredHoughLinesMarker);


    }
    else
    {
        ROS_WARN("Nothing to publish! No lines passed the filter!");
        return;
    }

}


void diane_octomap::DianeOctomapNodelet::PublishSequencedLinesSegments()
{
    if(SequencedLinesSegmentsPoints.size() > 0)
    {
        // init markers of occupied voxels:
        visualization_msgs::Marker SequencedLinesSegmentsMarker;

        SequencedLinesSegmentsMarker.header.frame_id = "/map";
        SequencedLinesSegmentsMarker.header.stamp = ros::Time::now();
        SequencedLinesSegmentsMarker.ns = "map";
        SequencedLinesSegmentsMarker.action = visualization_msgs::Marker::ADD;
        SequencedLinesSegmentsMarker.pose.orientation.w = 1.0;

        SequencedLinesSegmentsMarker.id = 25;

        SequencedLinesSegmentsMarker.type = visualization_msgs::Marker::LINE_LIST;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        SequencedLinesSegmentsMarker.scale.x = 0.01;

        SequencedLinesSegmentsMarker.color.r = 0.0;
        SequencedLinesSegmentsMarker.color.g = 0.0;
        SequencedLinesSegmentsMarker.color.b = 0.0;
        SequencedLinesSegmentsMarker.color.a = 1.0;


        for(int i=0; i<SequencedLinesSegmentsPoints.size(); i++)
        {
            geometry_msgs::Point line_point_init;
            line_point_init.x = SequencedLinesSegmentsPoints.at(i)(0,0);
            line_point_init.y = SequencedLinesSegmentsPoints.at(i)(1,0);
            line_point_init.z = SequencedLinesSegmentsPoints.at(i)(2,0);

            geometry_msgs::Point line_point_end;
            line_point_end.x = SequencedLinesSegmentsPoints.at(i)(0,1);
            line_point_end.y = SequencedLinesSegmentsPoints.at(i)(1,1);
            line_point_end.z = SequencedLinesSegmentsPoints.at(i)(2,1);


            SequencedLinesSegmentsMarker.points.push_back(line_point_init);
            SequencedLinesSegmentsMarker.points.push_back(line_point_end);

        }

        msgSequencedLinesSegmentsPub.publish(SequencedLinesSegmentsMarker);


    }
    else
    {
        ROS_WARN("Nothing to publish! No sequenced lines segments were detected!");
        return;
    }

}


void diane_octomap::DianeOctomapNodelet::PublishStairModelPoints()
{
    if(Modeled_Stairs.size() > 0)
    {
        size_t ModeledPointsSize = Modeled_Stairs.at(1)->Leafs_Of_Stair.cols();
        if (ModeledPointsSize <= 1)
        {
            ROS_WARN("Nothing to publish, modeled stair has no points stored.");
            return;
        }

        // init markers of occupied voxels:
        visualization_msgs::MarkerArray StairModelPoints;

        //Only has one array:
        StairModelPoints.markers.resize(1);

        //Configuring the color
        std_msgs::ColorRGBA _color; _color.r = (0.0); _color.g = (1.0); _color.b = (0.0); _color.a = 1.0;

        //Completing the StairModelPoints MarkerArray
        for(int i=0; i<Modeled_Stairs.at(1)->Leafs_Of_Stair.cols(); ++i)
        {
            float x = Modeled_Stairs.at(1)->Leafs_Of_Stair(0, i);
            float y = Modeled_Stairs.at(1)->Leafs_Of_Stair(1, i);
            float z = Modeled_Stairs.at(1)->Leafs_Of_Stair(2, i);

            //create marker:
            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            StairModelPoints.markers[0].points.push_back(cubeCenter);

            //Definindo a cor (Branco somente para visualizacão)
            StairModelPoints.markers[0].colors.push_back(_color);
        }

        ros::Time rostime = ros::Time::now();

        for(unsigned int j=0; j < StairModelPoints.markers.size(); ++j)
        {
            //Obtendo a quantidade de voxels na octree na profundidade indicada
            double size = Modeled_Stairs.at(1)->Leafs_Of_Stair.cols();

            StairModelPoints.markers[j].header.frame_id = "/map";
            StairModelPoints.markers[j].header.stamp = rostime;
            StairModelPoints.markers[j].ns = "map";
            StairModelPoints.markers[j].id = j;
            StairModelPoints.markers[j].type = visualization_msgs::Marker::CUBE_LIST;
            StairModelPoints.markers[j].scale.x = 0.05;
            StairModelPoints.markers[j].scale.y = 0.05;
            StairModelPoints.markers[j].scale.z = 0.05;

            if(StairModelPoints.markers[j].points.size() > 0)
            {
                StairModelPoints.markers[j].action = visualization_msgs::Marker::ADD;
            }
            else
            {
                StairModelPoints.markers[j].action = visualization_msgs::Marker::DELETE;
            }

        }

        msgStairModelPointsPub.publish(StairModelPoints);

    }
    else
    {
        ROS_WARN("Nothing to publish! No stairs detected on the map.");
        return;
    }

}




void diane_octomap::DianeOctomapNodelet::TreatBoolCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    //Publishing the occupied voxels of the octree stored.
    PublishOccupiedMarker();

    //Publishing the first filtered occupied voxels.
    PublishFirstFilteredOccupiedPoints();


    PublishHoughLines();


    PublishFilteredHoughLines();


    PublishSequencedLinesSegments();


    if(Modeled_Stairs.size() > 0)
    {
        //Publishing the markers of all modeled stairs, for visualization.
        PublishStairModelsVisual(Modeled_Stairs);

        //Publishing information about a modeled stair.
        PublishStairModel(Modeled_Stairs.at(0));

        //Publishing information about all modeled stairs.
        PublishAllStairsModel(Modeled_Stairs);


        //Publishing points of the detected stair.
        PublishStairModelPoints();

    }
    else
    {
        ROS_WARN("Nothing to publish! No stairs detected on the map.");
    }


}


void diane_octomap::DianeOctomapNodelet::TreatOctomapFullMapCallback(const Octomap::ConstPtr& msg)
{
    //Será chamado quando recebermos uma mensagem do octomap_server com a octree completa
    //Lendo a octree à partir da mensagem recebida (que está sendo publicada pelo Octomap_Server)
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
