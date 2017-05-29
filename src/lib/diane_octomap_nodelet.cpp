// diane_octomap_nodelet.cpp



#include <diane_octomap/diane_octomap_nodelet.h>


using octomap_msgs::Octomap;
using namespace std;


diane_octomap::DianeOctomapNodelet::DianeOctomapNodelet()
{
}


void diane_octomap::DianeOctomapNodelet::onInit()
{
    ///*********************************************
    ///Obtaining the parameters detection parameters
    ///*********************************************

    ros::NodeHandle & privateNH = getPrivateNodeHandle();


    ///Octomap's Parameters
    privateNH.param<string>("otFileName", otFileName, "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_video_objetos_5_final.ot");


    ///BoundingBox's Parameters
    privateNH.param<float>("BoundingBoxMinPointX", BoundingBoxMinPoint.x(), -5);
    privateNH.param<float>("BoundingBoxMinPointY", BoundingBoxMinPoint.y(), -5);
    privateNH.param<float>("BoundingBoxMinPointZ", BoundingBoxMinPoint.z(), 0.20);

    privateNH.param<float>("BoundingBoxMaxPointX", BoundingBoxMaxPoint.x(), 5);
    privateNH.param<float>("BoundingBoxMaxPointY", BoundingBoxMaxPoint.y(), 100);
    privateNH.param<float>("BoundingBoxMaxPointZ", BoundingBoxMaxPoint.z(), 100);


    ///Column Filter's Parameters
    privateNH.param<int>("ColumnMinSize", ColumnMinSize, 0);
    privateNH.param<int>("ColumnMaxSize", ColumnMaxSize, 6);


    ///Hough Transform's Parameters
    privateNH.param<double>("Rho_Passo", Rho_Passo, 0.05);
    privateNH.param<double>("Theta_Passo", Theta_Passo, 5);

    privateNH.param<double>("AccumulateDistTolerance", AccumulateDistTolerance, Rho_Passo/2);
    privateNH.param<int>("MinAmountVotes", MinAmountVotes, 10);


    ///Grouped Line's by (Rho, Theta) Parameters
    privateNH.param<int>("MinNumberLines", MinNumberLines, 3);
    privateNH.param<int>("MaxNumberLines", MaxNumberLines, 6);


    ///Segmentation's Parameters
    privateNH.param<double>("SegmentationXTol", SegmentationXTol, 0.11);


    ///Grouped Line's by (Theta, X-Interval) Parameters
    privateNH.param<double>("IntervalXTol", IntervalXTol, 0.15);


    ///Filtering Grouped Line's by quantity of Line Parameters
    privateNH.param<int>("MinNumSteps", MinNumSteps, 3);


    ///Merge Group's Lines by (Rho) Parameters
    privateNH.param<double>("MergeDeltaRhoTol", MergeDeltaRhoTol, 0.11);


    ///Sequence Filter's Parameters
    privateNH.param<double>("SequenceZMax", SequenceZMax, 0.30);
    privateNH.param<double>("SequenceMinStepWidth", SequenceMinStepWidth, 0.23);
    privateNH.param<double>("SequenceMaxStepWidth", SequenceMaxStepWidth, 0.38);


    ///Stair Modelling's Parameters
    privateNH.param<double>("ModellingMinStepHeight", ModellingMinStepHeight, 0.14);
    privateNH.param<double>("ModellingMaxStepHeight", ModellingMaxStepHeight, 0.23);
    privateNH.param<double>("ModellingMinStepWidth", ModellingMinStepWidth, 0.24);
    privateNH.param<double>("ModellingMaxStepWidth", ModellingMaxStepWidth, 0.36);




    ///*********************************************************************************
    ///Creating the Publishers/Subscribers/Services/Clients of the Diane Octomap Nodelet
    ///*********************************************************************************

    nodeHandle = getNodeHandle();


    ///Initializing the Publishers
//    msgOctomapFullMapPub = nodeHandle.advertise <Octomap> (getName() + "/octomap_full", 10, true);

//    msgOctomapFreeMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/free_cells_vis_array", 10, true);

    msgModeledStairPub = nodeHandle.advertise <diane_octomap::StairInfo>(getName() + "/Modeled_Stair_Info", 10);

    msgModeledStairAllPub = nodeHandle.advertise <diane_octomap::StairArrayInfo>(getName() + "/Modeled_Stairs_Info_All", 10);


    ///Initializing the Publishers needed for Visualization
    msgOctomapOccupiedMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/occupied_cells_vis_array", 10, true);

    msgOccupiedBoundingBoxMarkerPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/occupied_bounding_box_cells_vis_array", 10, true);

    msgFirstFilteredOccuppiedPointsPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/First_Filtered_Points", 10, true);

    msgHoughLinesPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Hough_Lines", 1000, true);

    msgFilteredHoughLinesPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Filtered_Hough_Lines", 1000, true);

    msgSequencedLinesSegmentsPub = nodeHandle.advertise <visualization_msgs::Marker> (getName() + "/Sequenced_Lines_Segments", 1000, true);

    msgStairModelPointsPub = nodeHandle.advertise <visualization_msgs::MarkerArray> (getName() + "/Stair_Model_Points", 10, true);

    msgModeledStairVisualPub = nodeHandle.advertise<visualization_msgs::Marker>(getName() + "/Modeled_Stairs_Visualization_Markers", 10);


    ///Initializing the Subscribers
    msgResetOctomapServerSub = nodeHandle.subscribe <std_msgs::Bool> (getName() + "/Start_Reset_Octomap_Server", 10, &DianeOctomapNodelet::TreatResetOctomapServerCallback, this);

    msgStartVisualizationPublishesSub = nodeHandle.subscribe <std_msgs::Bool> (getName() + "/Start_Visualization_Publishes", 10, &DianeOctomapNodelet::TreatStartVisualizationPublishesCallBack, this);


    ///Initializing the Services
    srvDetectStairsFromFileSer = nodeHandle.advertiseService(getName() + "/Detect_Stairs_From_File", &DianeOctomapNodelet::TreatDetectStairsFromFileCallback, this);
    srvDetectStairsFromServerSer = nodeHandle.advertiseService(getName() + "/Detect_Stairs_From_Server", &DianeOctomapNodelet::TreatDetectStairsFromServerCallback, this);


    ///Initializing the Clients
    srvResetOctomapServerCli = nodeHandle.serviceClient<std_srvs::Empty>("/Diane_Octomap/octomap_server_node/reset");
    srvRequestFullOctomapCli = nodeHandle.serviceClient<octomap_msgs::GetOctomap>("/Diane_Octomap/octomap_full");



    ///Inicializing the Thread's Cycle
    StartInternalCycle();

}


////Publishing the octree that is stored in the nodelet (If any other package wants to use it)
//void diane_octomap::DianeOctomapNodelet::PublishOctomapFullMap()
//{
//    Octomap map;
//    map.header.frame_id = "/map";
//    map.header.stamp = ros::Time::now();


//    if (octomap_msgs::fullMapToMsg(*octree, map))
//    {
//        msgOctomapFullMapPub.publish(map);
//    }
//    else
//    {
//        ROS_ERROR("Error serializing OctoMap");
//    }
//}



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


//Publishing a MarkerArray containing the occupied voxels
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
    std_msgs::ColorRGBA _color; _color.r = (0.62); _color.g = (0.66); _color.b = (0.85); _color.a = 1.0;

    for(OcTree::leaf_iterator it = octree->begin(), end = octree->end(); it!= end; ++it)
    {
        //If the node is occupied:
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

            //Defining the color (White for visulization)
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


//Publishing the occupied voxels that are inside the bounding box
void diane_octomap::DianeOctomapNodelet::PublishOccupiedBoundingBoxMarker()
{
    size_t occupiedBoundingBoxSize = OccupiedPoints.cols();

    if (occupiedBoundingBoxSize <= 1)
    {
        ROS_WARN("Nothing to publish. No voxels are contained inside the Bounding Box.");
        return;
    }


    // init markers of occupied voxels:
    visualization_msgs::MarkerArray OccupiedBoundingBoxNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    OccupiedBoundingBoxNodesVis.markers.resize(1);

    // now, traverse all leafs in the tree and completing the OccupiedMarker:
    std_msgs::ColorRGBA _color; _color.r = (0.42); _color.g = (0.43); _color.b = (0.75); _color.a = 1.0;

    //Completing the OccupiedBoundingBoxNodesVis MarkerArray
    for(int i=0; i<OccupiedPoints.cols(); ++i)
    {
        double x = OccupiedPoints(0, i);
        double y = OccupiedPoints(1, i);
        double z = OccupiedPoints(2, i);

        //create marker:
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        OccupiedBoundingBoxNodesVis.markers[0].points.push_back(cubeCenter);

        //Definindo a cor (Branco somente para visualizacão)
        OccupiedBoundingBoxNodesVis.markers[0].colors.push_back(_color);
    }

    ros::Time rostime = ros::Time::now();

    for(unsigned int j=0; j < OccupiedBoundingBoxNodesVis.markers.size(); ++j)
    {
        //Obtendo a quantidade de voxels na octree na profundidade indicada
        double size = OccupiedPoints.cols();

        OccupiedBoundingBoxNodesVis.markers[j].header.frame_id = "/map";
        OccupiedBoundingBoxNodesVis.markers[j].header.stamp = rostime;
        OccupiedBoundingBoxNodesVis.markers[j].ns = "map";
        OccupiedBoundingBoxNodesVis.markers[j].id = j;
        OccupiedBoundingBoxNodesVis.markers[j].type = visualization_msgs::Marker::CUBE_LIST;
        OccupiedBoundingBoxNodesVis.markers[j].scale.x = 0.05;
        OccupiedBoundingBoxNodesVis.markers[j].scale.y = 0.05;
        OccupiedBoundingBoxNodesVis.markers[j].scale.z = 0.05;

        if(OccupiedBoundingBoxNodesVis.markers[j].points.size() > 0)
        {
            OccupiedBoundingBoxNodesVis.markers[j].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            OccupiedBoundingBoxNodesVis.markers[j].action = visualization_msgs::Marker::DELETE;
        }

    }

    msgOccupiedBoundingBoxMarkerPub.publish(OccupiedBoundingBoxNodesVis);

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
    std_msgs::ColorRGBA _color; _color.r = (0.22); _color.g = (0.29); _color.b = (0.67); _color.a = 1.0;

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

        HoughLinesMarker.color.r = 0.90;
        HoughLinesMarker.color.g = 0.45;
        HoughLinesMarker.color.b = 0.45;
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

        FilteredHoughLinesMarker.color.r = 0.90;
        FilteredHoughLinesMarker.color.g = 0.22;
        FilteredHoughLinesMarker.color.b = 0.21;
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

        SequencedLinesSegmentsMarker.color.r = 0.72;
        SequencedLinesSegmentsMarker.color.g = 0.11;
        SequencedLinesSegmentsMarker.color.b = 0.11;
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
        // init markers of occupied voxels:
        visualization_msgs::MarkerArray StairModelPoints;

        for(int k=0; k<Modeled_Stairs.size(); ++k)
        {
            size_t ModeledPointsSize = Modeled_Stairs.at(k)->Leafs_Of_Stair.cols();
            if (ModeledPointsSize <= 1)
            {
                ROS_WARN("Nothing to publish, modeled stair has no points stored.");
                return;
            }


            //Only has one array:
            StairModelPoints.markers.resize(1);

            //Configuring the color
            std_msgs::ColorRGBA _color; _color.r = (0.0); _color.g = (0.78); _color.b = (0.36); _color.a = 1.0;

            //Completing the StairModelPoints MarkerArray
            for(int i=0; i<Modeled_Stairs.at(k)->Leafs_Of_Stair.cols(); ++i)
            {
                float x = Modeled_Stairs.at(k)->Leafs_Of_Stair(0, i);
                float y = Modeled_Stairs.at(k)->Leafs_Of_Stair(1, i);
                float z = Modeled_Stairs.at(k)->Leafs_Of_Stair(2, i);

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

        }

        msgStairModelPointsPub.publish(StairModelPoints);

    }
    else
    {
        ROS_WARN("Nothing to publish! No stairs detected on the map.");
        return;
    }

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




        // Line list is red (or is it?)
        line_list.color.r = 1.0;
        line_list.color.g = 0.42;
        line_list.color.b = 0.0;
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



    else
    {
        ROS_WARN("Nothing to publish! No stairs detected on the map.");
    }


}


void diane_octomap::DianeOctomapNodelet::TreatResetOctomapServerCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;

        if(srvResetOctomapServerCli.call(req, res))
        {
            cout << "Reset of Octomap Server worked!" << endl;
        }
        else
        {
            cout << "Reset did not work!" << endl;
        }

    }


}


void diane_octomap::DianeOctomapNodelet::TreatStartVisualizationPublishesCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    //Publishing the occupied voxels of the octree stored.
    PublishOccupiedMarker();


    PublishOccupiedBoundingBoxMarker();


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


bool diane_octomap::DianeOctomapNodelet::TreatDetectStairsFromFileCallback(std_srvs::Empty::Request & req , std_srvs::Empty::Response & res)
{
    ///Se recebeu uma requisicao para detectar a escada de um arquivo, inicializa as funcoes de deteccao da escada.


    ///Obtendo a octree à partir do arquivo.
    DianeOctomap::GenerateOcTreeFromFile();


    ///Filtrando e armazenando as folhas da octree que estejam dentro da Bounding Box definida no método e que estejam ocupadas.
    DianeOctomap::GetOccupiedLeafsOfBBX(octree);


    ///Utilizando as folhas filtradas (presentes no vetor) para detectar as informacões da escada.
    //DianeOctomap::StairDetection();
    DianeOctomap::StairDetection2D();


    return true;

}


bool diane_octomap::DianeOctomapNodelet::TreatDetectStairsFromServerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ///Se recebeu uma requisicao para detectar a escada online, inicializa as funcoes de deteccao da escada.
    bool result = false;

    ///Aplicando um Reset no Octomap Server para que as informações obtidas do mapa sejam referentes às informações atuais dos sensores;
    if(ResetOctomapServer())
    {
        ///Obtendo a octree que está armazenada no octomap_server (não será mais obtida de um arquivo) ---> A ser testado com um Kinect (incluir validacão de octree vazia)
        if(RequestOctomapServerOctree())
        {
            ///Filtrando e armazenando as folhas da octree que estejam dentro da Bounding Box definida no método e que estejam ocupadas.
            DianeOctomap::GetOccupiedLeafsOfBBX(octree);

            ///Utilizando as folhas filtradas (presentes no vetor) para detectar as informacões da escada.
            //DianeOctomap::StairDetection();
            DianeOctomap::StairDetection2D();

            result = true;

        }

    }

    return result;
}


bool diane_octomap::DianeOctomapNodelet::ResetOctomapServer()
{
    ///Aplicando um Reset no Octomap Server para que as informações obtidas do mapa sejam referentes às informações atuais dos sensores;
    bool resetResult = false;

    std_srvs::EmptyRequest ResetOctomapServerReq;
    std_srvs::EmptyResponse ResetOctomapServerRes;

    if(srvResetOctomapServerCli.call(ResetOctomapServerReq, ResetOctomapServerRes))
    {
        resetResult = true;
    }

    return resetResult;

}


bool diane_octomap::DianeOctomapNodelet::RequestOctomapServerOctree()
{
    ///Obtendo a octree que está armazenada no octomap_server (não será mais obtida de um arquivo) ---> A ser testado com um Kinect (incluir validacão de octree vazia)
    bool result = false;

    octomap_msgs::GetOctomapRequest OcTreeReq;
    octomap_msgs::GetOctomapResponse OcTreeRes;

    AbstractOcTree* abs_tree;

    if(srvRequestFullOctomapCli.call(OcTreeReq, OcTreeRes))
    {
        abs_tree = octomap_msgs::msgToMap(OcTreeRes.map);

        //octreeFromMsg = dynamic_cast<OcTree*>(abs_tree);
        octree = dynamic_cast<OcTree*>(abs_tree);

        Octree_Resolution = octree->getResolution();

        result = true;
    }

    return result;

}


diane_octomap::DianeOctomapNodelet::~DianeOctomapNodelet()
{
    StopInternalCycle();
}
