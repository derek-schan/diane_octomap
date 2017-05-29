// diane_octomap.cpp

#include <iostream>
#include <fstream>

#include <diane_octomap/diane_octomap.h>

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;


diane_octomap::DianeOctomap::DianeOctomap()
{
    stop = false;

    ///Octomap's Variables (OcTree Structures; Properties; and Structures containing Leafs of Bounding Box; and path to OcTree file)
    OccupiedPoints = MatrixXf();


//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_1.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_1.ot";

//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_3.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_3.ot";

//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_5.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_5.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_5_2.ot";

//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_video_objetos_5_final.ot";

//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Principal_5.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_LEAD_5.ot";


//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_7.ot";
//    otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_7.ot";



    ///BoundingBox's Variables
//    BoundingBoxMinPoint.x() = -5;   //-5
//    BoundingBoxMinPoint.y() = -5;   //-5
//    BoundingBoxMinPoint.z() = 0.20;  //0.05

//    BoundingBoxMaxPoint.x() = 5;    //5
//    BoundingBoxMaxPoint.y() = 100;  //100
//    BoundingBoxMaxPoint.z() = 100;  //100

    ///Column Filter's Variables (Group by X,Y and filter)
//    ColumnMinSize = 0;  //0
//    ColumnMaxSize = 6;  //6


    ///Hough Transform's Variables
//    Rho_Passo = 0.05;   //0.05
//    Theta_Passo = 5;    //5

//    AccumulateDistTolerance = Rho_Passo/2;  //0.025
//    MinAmountVotes = 10;    //10


    ///Grouped Line's by (Rho, Theta) Variables
//    MinNumberLines = 3; //3
//    MaxNumberLines = 6; //6


    ///Segmentation's Variables
//    SegmentationXTol = 0.11; //0.11


    ///Grouped Line's by (Theta, X-Interval) Variables
//    IntervalXTol = 0.15;    //0.11


    ///Filtering Grouped Line's by quantity of Line Variables
//    MinNumSteps = 3;    //3


    ///Merge Group's Lines by (Rho) Variables
//    MergeDeltaRhoTol = 0.11;    //0.11


    ///Sequence Filter's Variables
//    SequenceZMax = 0.30;    //0.10
//    SequenceMinStepWidth = 0.23;    //0.25
//    SequenceMaxStepWidth = 0.38;    //0.35


    ///Stair Modelling's Variables
//    ModellingMinStepHeight = 0.14;  //0.14
//    ModellingMaxStepHeight = 0.23;  //0.23
//    ModellingMinStepWidth = 0.24;   //0.24
//    ModellingMaxStepWidth = 0.36;   //0.36





    ///Variables used for publishing in messages/services
    First_Filtered_Points = MatrixXf();


//    ///Variáveis para filtrar os planos a serem extraídos na Transformada de Hough em 3D (Deteccão de planos)
//    Filter_Phi_Min = 85;
//    Filter_Phi_Max = 96;
//    Filter_Vote_Min = 250;
//    Filter_Vote_Max = 400;


//    ///Definindo as tolerâncias para o merge dos objetos Line's (Se a distância estiver dentro da tolerância, aplica o merge)
//    delta_merge_Rho = 0.11; //0.11

//    ///Definindo as tolerâncias para o merge dos Planos detectados (Somente utilizado no 3D)
//    delta_Rho = 0.1;
//    delta_Theta = 0;
//    delta_Phi = 0;






}


void diane_octomap::DianeOctomap::onInit()
{
    StartInternalCycle();
}


void diane_octomap::DianeOctomap::StartInternalCycle()
{
    mutStartStop.lock();

    stop = false;


    internalThread = new boost::thread(DianeOctomap::InternalThreadFunction, this);

    mutStartStop.unlock();

}


void diane_octomap::DianeOctomap::StopInternalCycle()
{
    mutStartStop.lock();

    stop = true;
    internalThread->join();
    delete internalThread;
    internalThread = NULL;

    mutStartStop.unlock();

}


void diane_octomap::DianeOctomap::InternalThreadFunction(DianeOctomap* diane_mapper)
{
    diane_mapper->InternalCycleProcedure();
}


void diane_octomap::DianeOctomap::InternalCycleProcedure()
{
    while (!stop)
    {

    }
}


//Gerando a octree à partir de um file (Pode-se substituir para ler uma octree do octomap_server - Será necessário para uma deteccão online)
void diane_octomap::DianeOctomap::GenerateOcTreeFromFile()
{
    AbstractOcTree* abs_tree = AbstractOcTree::read(otFileName);
    if(abs_tree) // read error returns NULL
    {
        octree = dynamic_cast<OcTree*>(abs_tree);

        if (octree) // cast succeeds if correct type
        {
            //Expandindo a octree
            octree->expand();

            Octree_Resolution = octree->getResolution();

            size_t occupied_count(0);

            size_t free_count(0);

            size_t total_count(0);

            double size;

            for(OcTree::leaf_iterator it = octree->begin(), end=octree->end(); it!= end; ++it)
            {
                if(octree->isNodeOccupied(*it))
                {
                    occupied_count++;
                }
                else
                {
                    free_count++;
                }

                total_count++;

                size = it.getSize();
            }

            cout << "\nWriting OcTree Information\n===========================\n\n" << endl;

            cout << "Número de nós ocupados: " << occupied_count << ".\n" << endl;
            cout << "Número de nós livres: " << free_count << ".\n" << endl;
            cout << "Número de nós totais: " << total_count << ".\n" << endl;

        }

    }

}


///Method that stores in the OccupiedPoints structure the coordinates of the occupied leafs
void diane_octomap::DianeOctomap::GetOccupiedLeafsOfBBX(OcTree* octree)
{
    //Variable that stores the number of points that are inside the Bounding Box
    int Occupied_Points_Count = 0;

    OccupiedPoints.conservativeResize(3, octree->getNumLeafNodes());


    //Storing the coordinates of the occupied leafs contained inside the Bounding Box.
    for(OcTree::leaf_bbx_iterator bbx_it = octree->begin_leafs_bbx(BoundingBoxMinPoint, BoundingBoxMaxPoint), end=octree->end_leafs_bbx(); bbx_it!= end; ++bbx_it)
    {
        if(octree->isNodeOccupied(*bbx_it))
        {
            //Storing the leaf in the vector (Probably unused)
            OccupiedLeafsInBBX.push_back(bbx_it);

            //Storing the leaf's coordinates in the Matrix
            OccupiedPoints(0, Occupied_Points_Count) = (double)bbx_it.getX();
            OccupiedPoints(1, Occupied_Points_Count) = (double)bbx_it.getY();
            OccupiedPoints(2, Occupied_Points_Count) = (double)bbx_it.getZ();

            ++Occupied_Points_Count;

        }
    }

    OccupiedPoints.conservativeResize(3, Occupied_Points_Count);

    cout << "Folhas ocupadas: " << Occupied_Points_Count << endl << endl;

}



///-------------------------------------------------- Stair Detection Method using 2D logic ---------------------------------------------------------

void diane_octomap::DianeOctomap::StairDetection2D()
{
    ///Grouping the points by (X,Y) coordinates
    vector<MatrixXf> GroupedXY = GroupLeafsByXY(OccupiedPoints);


    ///Group the points that passed the column filter by Z coordinate
    vector<MatrixXf> Grouped_Leafs_In_Matrix = GroupLeafsByZ(GroupedXY);


    ///Method that extract and stores the points that passed the column filter
    ExtractColumnFilteredPoints(GroupedXY);


    ///Hough - Selecting the straight lines in each Z-Plane
    //Obtaining the space parameter
    GetSpaceParameters(OccupiedPoints);


    //Executing the Hough's Transform in each Group of Leafs (grouped by Z)
    vector<vector<diane_octomap::Line*>> Lines = LineHoughTransform(Grouped_Leafs_In_Matrix);


    ///Method that obtains and stores the points of the Line's received from Hough's Transform
    ExtractHoughLinesPoints(Lines);


    ///Grouping the Line's objects that have the same Rho and Theta Parameters
    vector<vector<diane_octomap::Line*>> GroupLinesByRhoTheta = GroupLineByRhoTheta(Lines);


    ///Filtering groups of Line's which sizes exceeds a maximum amount or does not surpass a minimum amount
    //(If the size does not surpass a minimum amount, it is considered a noise. If the size exceeds a maximum amount, it is considered a wall)
    vector<vector<diane_octomap::Line*>> Filtered_Groups = FilterGroups(GroupLinesByRhoTheta);


    ///Method that obtains and stores the points of the Filtered Line's
    ExtractFilteredHoughLinesPoints(Filtered_Groups);


    ///Merging the Line's contained in a Group, that passed the occurrency filter.
    ///The resultant Line object of each group keeps the minimum and maximum values of the Z-coordinate, in which the line was detected.
    vector<diane_octomap::Line*> Merged_Lines = MergeGroupedLines(Filtered_Groups);


    ///Populating the Line's objects with the leaf's informations
    PopulateLinesWithMatrix(Merged_Lines, Grouped_Leafs_In_Matrix);


    ///Segmenting the Line's by it's leafs (sorting the leafs and segmenting)
    vector<Line*> Segmented_Lines_With_Matrix = SegmentLinesWithMatrix(Merged_Lines);


    ///Grouping the Line s objects by (Theta, X-Interval of the segment)
    vector<vector<Line*>> GroupThetaIntervalLines = GroupLinesByThetaAndInterval(Segmented_Lines_With_Matrix);


    ///Filtering Groups of Line's that does not contains a number of number sufficient to generate a stairway
    vector<vector<Line*>> Filtered_Segmented_Groups = FilterGroupedLines(GroupThetaIntervalLines);


    ///Merging the Line's contained in a group by Rho parameter
    vector<vector<Line*>> Merged_Segmented_Groups = MergeSegmentedGroupsLines(Filtered_Segmented_Groups);


    ///Finding a valid sequence inside the Group of Lines
    vector<vector<Line*>> Filtered_Sequence = SequenceFilter(Merged_Segmented_Groups);


    ///Method that obtains and stores the points of the Sequences Line's Segments
    ExtractSequencedLinesSegmentsPoints(Filtered_Sequence);


    ///Creating the Stair candidates objects from the groups of Line's
    //Each Group represents a stairway and each Line in a group represents a Step
    vector<Stair*> StairCandidates = CreateStairCandidatesWithMatrix(Filtered_Sequence);


    ///Encontrando Rho's e Theta's melhores para cada escada, utilizando mínimos quadrados sobre as folhas dessas Line's
    ///Discovering optimized Rho's and Theta's parameters for each Stair, applying Least Square to the Line's Leafs
    UpdateStairProperties(StairCandidates);


    ///Modelando cada candidato à escada (obtendo o comprimento, largura e altura média dos degraus, a aresta inicial e os pontos de arestas referentes aos outros degraus)
    Modeled_Stairs = ModelStairsWithMatrix(StairCandidates);

//    WriteModeledStairPropertiesToFile(Modeled_Stairs.at(0));
}




//-----------------------------------------Métodos utilizando Matriz-----------------------------------------------
vector<MatrixXf> diane_octomap::DianeOctomap::GroupLeafsByXY(MatrixXf Leafs)
{
    //Vector containing the columns of points (grouped by (X,Y) coordinates)
    vector<MatrixXf> GroupedXY;

    //Temporary vector containing the columns of points (grouped by (X,Y) coordinates)
    vector<MatrixXf> GroupedXYTemp;


    for(int i=0; i<Leafs.cols(); ++i)
    {
        bool group_not_found = true;

        //Finding the leaf's group (same (X,Y) coordinates)
        for(int j=0; j<GroupedXYTemp.size(); ++j)
        {
            if(GroupedXYTemp.at(j)(0,0) == Leafs(0,i) && GroupedXYTemp.at(j)(1,0) == Leafs(1,i))
            {
                GroupedXYTemp.at(j).conservativeResize(3, GroupedXYTemp.at(j).cols() +1);
                GroupedXYTemp.at(j)(0, GroupedXYTemp.at(j).cols() - 1) = Leafs(0, i);
                GroupedXYTemp.at(j)(1, GroupedXYTemp.at(j).cols() - 1) = Leafs(1, i);
                GroupedXYTemp.at(j)(2, GroupedXYTemp.at(j).cols() - 1) = Leafs(2, i);
                group_not_found = false;
                break;
            }
        }

        //If no group was found, create a new one
        if(group_not_found)
        {
            MatrixXf new2DMatrix = MatrixXf(3, 1);
            new2DMatrix(0,0) = Leafs(0, i);
            new2DMatrix(1,0) = Leafs(1, i);
            new2DMatrix(2,0) = Leafs(2, i) ;
            GroupedXYTemp.push_back(new2DMatrix);
        }

    }

    //Filtering the groups by column size
    for(int k=0; k<GroupedXYTemp.size(); ++k)
    {
        MatrixXf LeafColumn = GroupedXYTemp.at(k);

        if((LeafColumn.cols()>=ColumnMinSize) && (LeafColumn.cols()<=ColumnMaxSize))
        {
            GroupedXY.push_back(LeafColumn);
        }

    }

    return GroupedXY;

}


vector<MatrixXf> diane_octomap::DianeOctomap::GroupLeafsByZ(vector<MatrixXf> LeafColumns)
{
    //Vector containing the points that have the same Z coordinate
    vector<MatrixXf> Grouped2D;


    for(int i=0; i<LeafColumns.size(); ++i)
    {
        for(int j=0; j<LeafColumns.at(i).cols(); ++j)
        {
            bool group_not_found = true;

            //Finding the leaf's group (same Z coordinates)
            for(int k=0; k<Grouped2D.size(); ++k)
            {
                if(Grouped2D.at(k)(2,0) == LeafColumns.at(i)(2, j))
                {
                    Grouped2D.at(k).conservativeResize(3, Grouped2D.at(k).cols() + 1);
                    Grouped2D.at(k)(0 , Grouped2D.at(k).cols() - 1) = LeafColumns.at(i)(0, j);
                    Grouped2D.at(k)(1 , Grouped2D.at(k).cols() - 1) = LeafColumns.at(i)(1 ,j);
                    Grouped2D.at(k)(2 , Grouped2D.at(k).cols() - 1) = LeafColumns.at(i)(2 ,j);
                    group_not_found = false;
                    break;
                }

            }

            //If no group was found, create a new one
            if(group_not_found)
            {
                MatrixXf new2DMatrix = MatrixXf(3, 1);
                new2DMatrix(0,0) = LeafColumns.at(i)(0, j);
                new2DMatrix(1,0) = LeafColumns.at(i)(1, j);
                new2DMatrix(2,0) = LeafColumns.at(i)(2, j);
                Grouped2D.push_back(new2DMatrix);
            }

        }

    }

    return Grouped2D;

}


void diane_octomap::DianeOctomap::GetSpaceParameters(MatrixXf Leafs)
{
    double max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;

    for(int i=0; i<Leafs.cols(); i++)
    {

        if(Leafs(0, i) < min_x)
        {
            min_x = Leafs(0, i);
        }
        if(Leafs(0, i) > max_x)
        {
            max_x = Leafs(0, i);
        }

        if(Leafs(1, i) < min_y)
        {
            min_y = Leafs(1, i);
        }
        if(Leafs(1, i) > max_y)
        {
            max_y = Leafs(1, i);
        }

    }


    SpaceLength = max_x - min_x;
    SpaceWidth = max_y - min_y;

    SpaceXMin = min_x;
    SpaceXMax = max_x;
    SpaceYMin = min_y;
    SpaceYMax = max_y;

}



vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::LineHoughTransform(vector<MatrixXf> Leafs)
{
    vector<vector<Line*>> Group_Lines;

    //Parameters used to generate the Hough's Space
    Rho_Min = 0;
    Rho_Max = ceil(sqrt(pow(SpaceLength, 2) + pow(SpaceWidth, 2)));
    Theta_Min = 0;
    Theta_Max = 360;

    //Calculating the number of divisions on the Rho Axis and Theta Axis
    Rho_Num = (Rho_Max - Rho_Min)/Rho_Passo;
    Theta_Num = (Theta_Max - Theta_Min)/Theta_Passo;

    //Executing the Hough Transform for each group
    for (int i=0; i<Leafs.size(); i++)
    {
        vector<vector<int>> Votes = Accumulation2D(Leafs.at(i));

        vector<Line*> Lines = CreateGroupLines(Votes, Leafs.at(i)(2,0));

        Group_Lines.push_back(Lines);
    }

    return Group_Lines;

}


vector<vector<int>> diane_octomap::DianeOctomap::Accumulation2D(MatrixXf LeafZ)
{
    vector<vector<int>> Accumulate2d;
    vector<int> addAccumulate;
    bool add = false;

    //Creating the Hough's Space
    for(int i=0; i<Rho_Num ; i++)
    {
        if(add == false)
        {
            for(int j=0; j<Theta_Num; j++)
            {
                addAccumulate.push_back(0);
            }

            add=true;
        }

        Accumulate2d.push_back(addAccumulate);

    }

    //Voting process using the points in the matrix
    for (int k=0; k<Theta_Num; k++)
    {
        float Theta = (Theta_Min + k*Theta_Passo + Theta_Passo/2) * (M_PI/180);
        float ctheta = cos(Theta);
        float stheta = sin(Theta);

        for(int l=0; l<Rho_Num; l++)
        {
            float Rho = (Rho_Min + (l+0.5)*Rho_Passo);

            for (int m=0; m<LeafZ.cols(); m++)
            {
                float Rho_point = LeafZ(0, m)*ctheta + LeafZ(1, m)*stheta;

                if (fabs(Rho_point - Rho) <= AccumulateDistTolerance)
                {
                    Accumulate2d.at(l).at(k) = Accumulate2d.at(l).at(k)+ 1;
                }

            }

        }

    }

    return Accumulate2d;

}


//Creating the Line's objects, considering the voting process results. Returns every Line that received a minimum amount of votes.
vector<diane_octomap::Line*> diane_octomap::DianeOctomap::CreateGroupLines(vector<vector<int>> Votes, float Z)
{
    vector<Line*> Lines;

    for (int i=0; i<Votes.size(); i++)
    {
        for(int j=0; j<Votes.at(i).size(); j++)
        {
            //Create the Line object, if it received a minimum amount of votes
            if(Votes.at(i).at(j) > MinAmountVotes)
            {
                Line* line = new Line();
                line->Line_Rho = (Rho_Min + (i+0.5) * Rho_Passo);
                line->Line_Theta = (Theta_Min + (j+0.5) * Theta_Passo);
                line->Line_Votes = Votes.at(i).at(j);
                line->Line_Z = Z;

                Lines.push_back(line);
            }

        }

    }

    return Lines;

}


vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::GroupLineByRhoTheta(vector<vector<diane_octomap::Line*>> Lines)
{
    vector<vector<diane_octomap::Line*>>GroupedLineByTheta;

    double Theta;
    double Rho;

    //Grouping the Line's by Rho and Theta parameters
    for(int i=0; i<Lines.size(); i++)
    {
        for (int k=0; k<Lines.at(i).size(); k++)
        {
             Theta = Lines.at(i).at(k)->Line_Theta;
             Rho = Lines.at(i).at(k)->Line_Rho;

             bool Group_Found = false;

             //Verify if (Rho, Theta) parameters belongs to a created group.
             for(int j=0; j<GroupedLineByTheta.size(); j++)
             {
                 if((Theta == GroupedLineByTheta.at(j).at(0)->Line_Theta) && (Rho == GroupedLineByTheta.at(j).at(0)->Line_Rho))
                 {
                     GroupedLineByTheta.at(j).push_back(Lines.at(i).at(k));

                     Group_Found = true;

                     break;
                 }

             }

             //If no group was found, create a new group
             if(Group_Found == false)
             {
                 vector<diane_octomap::Line*> NewGroup;
                 NewGroup.push_back(Lines.at(i).at(k));

                 GroupedLineByTheta.push_back(NewGroup);
             }

         }

    }

    return GroupedLineByTheta;

}


//Filtering groups of Line's which sizes exceeds a maximum amount or does not surpass a minimum amount
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::FilterGroups(vector<vector<diane_octomap::Line*>> GroupLineByTheta)
{
    vector<vector<diane_octomap::Line*>> FilterGroups;

    for (int i=0; i<GroupLineByTheta.size(); i++)
    {
        if((GroupLineByTheta.at(i).size() >= MinNumberLines) && (GroupLineByTheta.at(i).size() <= MaxNumberLines))
        {
            FilterGroups.push_back(GroupLineByTheta.at(i));
        }
    }

    return FilterGroups;

}


vector<diane_octomap::Line*> diane_octomap::DianeOctomap::MergeGroupedLines(vector<vector<diane_octomap::Line*>> GroupedLines)
{
    vector<Line*> Merged_Lines;

    for(int i=0; i<GroupedLines.size(); i++)
    {
        vector<Line*> Group_Line = GroupedLines.at(i);

        Line* New_Line = new Line();

        for(int j=0; j<Group_Line.size(); j++)
        {
            Line* line = Group_Line.at(j);

            New_Line->Line_Rho = line->Line_Rho;
            New_Line->Line_Theta = line->Line_Theta;

            New_Line->Line_Votes += line->Line_Votes;

            if(New_Line->min_Z > line->Line_Z)
            {
                New_Line->min_Z = line->Line_Z;
            }

            if(New_Line->max_Z < line->Line_Z)
            {
                New_Line->max_Z = line->Line_Z;
            }

        }

        Merged_Lines.push_back(New_Line);

    }

    return Merged_Lines;

}




void diane_octomap::DianeOctomap::PopulateLinesWithMatrix(vector<diane_octomap::Line*>& Merged_Lines, vector<MatrixXf> Leaf_Groups_In_Matrix)
{
    for(int i=0; i<Merged_Lines.size(); i++)
    {
        double Rho = Merged_Lines.at(i)->Line_Rho;
        double Theta = Merged_Lines.at(i)->Line_Theta * (M_PI/180);

        double ctheta = cos(Theta);
        double stheta = sin(Theta);

        for(int j=0; j<Leaf_Groups_In_Matrix.size(); j++)
        {
            MatrixXf Leaf_Group = Leaf_Groups_In_Matrix.at(j);

            //Verify if the Group of Leaf's Z-coordinate is contained inside the Line's limits of Z.
            if((Leaf_Group(2, 0)  >= Merged_Lines.at(i)->min_Z) && (Leaf_Group(2, 0)  <= Merged_Lines.at(i)->max_Z))
            {
                for(int k=0; k<Leaf_Group.cols(); k++)
                {
                    Vector3f Accessed_Leaf = Vector3f(3, 1);
                    Accessed_Leaf(0, 0) = Leaf_Group(0, k);
                    Accessed_Leaf(1, 0) = Leaf_Group(1, k);
                    Accessed_Leaf(2, 0) = Leaf_Group(2, k);

                    double Rho_Point = (Accessed_Leaf(0, 0)*ctheta + Accessed_Leaf(1, 0)*stheta);

                    if(fabs(Rho_Point - Rho) <= Rho_Passo/2)
                    {
                        //If the Line's leaf matrix is empty, includes the new leaf.
                        if(Merged_Lines.at(i)->Leafs_Of_Line.cols() == 0)
                        {
                            int Actual_Size = Merged_Lines.at(i)->Leafs_Of_Line.cols();
                            Merged_Lines.at(i)->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                            //Including the leaf's informations.
                            Merged_Lines.at(i)->Leafs_Of_Line(0, Actual_Size) = Accessed_Leaf(0, 0);
                            Merged_Lines.at(i)->Leafs_Of_Line(1, Actual_Size) = Accessed_Leaf(1, 0);
                            Merged_Lines.at(i)->Leafs_Of_Line(2, Actual_Size) = Accessed_Leaf(2, 0);

                        }
                        else
                        {
                            //If it is not empty, check if the leaf is not contained in the group.
                            if((GroupContainsLeaf(Merged_Lines.at(i)->Leafs_Of_Line, Accessed_Leaf)) == false)
                            {
                                int Actual_Size = Merged_Lines.at(i)->Leafs_Of_Line.cols();
                                Merged_Lines.at(i)->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                                //Including the leaf's information.
                                Merged_Lines.at(i)->Leafs_Of_Line(0, Actual_Size) = Accessed_Leaf(0, 0);
                                Merged_Lines.at(i)->Leafs_Of_Line(1, Actual_Size) = Accessed_Leaf(1, 0);
                                Merged_Lines.at(i)->Leafs_Of_Line(2, Actual_Size) = Accessed_Leaf(2, 0);

                            }

                        }


                    }

                }

            }

        }

        //After the population process ends, update the X and Z limits of the Line.
        Merged_Lines.at(i)->UpdateLimitsWithMatrix();

    }

}


bool diane_octomap::DianeOctomap::GroupContainsLeaf(MatrixXf Leaf_Group, Vector3f Leaf)
{
    auto find_row_X = (Leaf_Group.row(0).array() == Leaf[0]);
    auto find_row_Y = (Leaf_Group.row(1).array() == Leaf[1]);
    auto find_row_Z = (Leaf_Group.row(2).array() == Leaf[2]);

    auto merge_result = find_row_X * find_row_Y * find_row_Z;

    bool contains = merge_result.any();

    return contains;
}


vector<diane_octomap::Line*> diane_octomap::DianeOctomap::SegmentLinesWithMatrix(vector<Line*> Lines)
{
    vector<Line*> NewLines;
    vector<Line*> TempLines;


    bool NewLine_break = false;

    int i;
    for(i=0; i<Lines.size(); i++)
    {
        Line* Accessed_Line = Lines.at(i);

        //Sorting the matrix of leafs by X-coordinate
        Accessed_Line->SortLeafMatrixByX();


        //Foreach leaf, verify if the distance in X to the next leaf surpass the tolerance
        for(int j=0; j<Accessed_Line->Leafs_Of_Line.cols() - 1; j++)
        {
            if((fabs(Accessed_Line->Leafs_Of_Line(0, j) - Accessed_Line->Leafs_Of_Line(0, j+1)) > SegmentationXTol ))
            {
                NewLine_break = true;

                Line* NewLine1 = new Line();
                Line* NewLine2 = new Line();

                NewLine1->Line_Rho = Accessed_Line->Line_Rho;
                NewLine1->Line_Theta = Accessed_Line->Line_Theta;

                NewLine2->Line_Rho = Accessed_Line->Line_Rho;
                NewLine2->Line_Theta = Accessed_Line->Line_Theta;

                //Storing the leafs of the first segment on the first new Line object
                for(int k=0; k<=j; k++)
                {
                    int Actual_Size = NewLine1->Leafs_Of_Line.cols();
                    NewLine1->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                    NewLine1->Leafs_Of_Line(0, k) = Accessed_Line->Leafs_Of_Line(0, k);
                    NewLine1->Leafs_Of_Line(1, k) = Accessed_Line->Leafs_Of_Line(1, k);
                    NewLine1->Leafs_Of_Line(2, k) = Accessed_Line->Leafs_Of_Line(2, k);
                }

                //Storing the leafs of the second segment on the second new Line object
                for(int l=j+1; l<Accessed_Line->Leafs_Of_Line.cols(); l++)
                {
                    int Actual_Size = NewLine2->Leafs_Of_Line.cols();
                    NewLine2->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                    NewLine2->Leafs_Of_Line(0, Actual_Size) = Accessed_Line->Leafs_Of_Line(0, l);
                    NewLine2->Leafs_Of_Line(1, Actual_Size) = Accessed_Line->Leafs_Of_Line(1, l);
                    NewLine2->Leafs_Of_Line(2, Actual_Size) = Accessed_Line->Leafs_Of_Line(2, l);
                }

                //Updating the Votes of the Line's
                NewLine1->Line_Votes = NewLine1->Leafs_Of_Line.cols();
                NewLine2->Line_Votes = NewLine2->Leafs_Of_Line.cols();

                //Updating the limits of the Line's
                NewLine1->UpdateLimitsWithMatrix();
                NewLine2->UpdateLimitsWithMatrix();

                //Sorting the leafs by X-coordinate
                NewLine1->SortLeafMatrixByX();
                NewLine2->SortLeafMatrixByX();

                TempLines.push_back(NewLine1);
                TempLines.push_back(NewLine2);

                break;

            }

        }

        //If a segmentation happened for a Line, breaks the loop and calls the recursive function again.
        if(NewLine_break)
        {
            break;
        }
        else
        {
            TempLines.push_back(Accessed_Line);
        }

    }

    //If a break happened, complete the list with the rest of the Line's and calls the recursive function again.
    if(NewLine_break)
    {
        for(int m=i+1; m<Lines.size(); m++)
        {
            TempLines.push_back(Lines.at(m));
        }

        NewLines = SegmentLinesWithMatrix(TempLines);

    }
    else
    {
        NewLines = TempLines;
    }


    return NewLines;

}


//Grouping the Line's objects by the same Theta and Limits of X (min_X and max_X properties)
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::GroupLinesByThetaAndInterval(vector<diane_octomap::Line*> Segmented_Lines)
{
    vector<vector<Line*>> Groups_Lines;

    for(int i=0; i<Segmented_Lines.size(); i++)
    {
        Line* line = Segmented_Lines.at(i);
        double line_rho = line->Line_Rho;
        double line_theta = line->Line_Theta;   //In degrees

        bool group_found = false;

        for(int j=0; j<Groups_Lines.size(); j++)
        {
            //If the Line's Theta property equals a Group's Theta property, verify if the Line segment has X limits consistent with the X limits of the first Line of the Group.
            if(line_theta == Groups_Lines.at(j).at(0)->Line_Theta)
            {
                //Obtaining the normal of the line (needed for verification of the X interval position of the first line)
                double line_normal[2];
                line_normal[0] = cos(line_theta * (M_PI/180));
                line_normal[1] = sin(line_theta * (M_PI/180));

                double dist = fabs(line_rho - Groups_Lines.at(j).at(0)->Line_Rho);

                double temp_min_X = line->min_X - (dist*line_normal[0]);
                double temp_max_X = line->max_X - (dist*line_normal[0]);


                //If the distances between the minimum values of the intervals and between the maximum values of the intervals are inside the tolerance, the new Line is added in the Group.
                if((fabs(temp_min_X - Groups_Lines.at(j).at(0)->min_X) <= IntervalXTol) && (fabs(temp_max_X - Groups_Lines.at(j).at(0)->max_X) <= IntervalXTol))
                {
                    Groups_Lines.at(j).push_back(line);
                    group_found = true;

                    break;
                }

            }

        }

        //If no valid group was found, creates a new one.
        if(group_found == false)
        {
            vector<Line*> Group_Lines;
            Group_Lines.push_back(line);

            //Adding a new group
            Groups_Lines.push_back(Group_Lines);
        }

    }

    return Groups_Lines;

}


vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::FilterGroupedLines(vector<vector<diane_octomap::Line*>> GroupThetaIntervalLines)
{
    vector<vector<Line*>> Filtered_Segmented_Groups;

    for(int i=0; i<GroupThetaIntervalLines.size(); i++)
    {
        if(GroupThetaIntervalLines.at(i).size() >= MinNumSteps)
        {
            Filtered_Segmented_Groups.push_back(GroupThetaIntervalLines.at(i));
        }
    }

    return Filtered_Segmented_Groups;

}




vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::MergeSegmentedGroupsLines(vector<vector<diane_octomap::Line*>> GroupThetaIntervalLines)
{
    vector<vector<Line*>> Merged_Segmented_Lines;

    //Apply the merge for each group
    for(int i=0; i<GroupThetaIntervalLines.size(); i++)
    {
        vector<Line*> Merged_Lines = MergeSegmentedGroup(GroupThetaIntervalLines.at(i));

        if(Merged_Lines.size() >= MinNumSteps)
        {
            //Sorting the Line's of a group by Rho
            vector<Line*> Sorted_Lines = SortGroupLines(Merged_Lines);

            //Storing the group of Line's that passed through the Merge and Sort
            Merged_Segmented_Lines.push_back(Sorted_Lines);
        }

    }

    return Merged_Segmented_Lines;
}


vector<diane_octomap::Line*> diane_octomap::DianeOctomap::MergeSegmentedGroup(vector<diane_octomap::Line*> SegmentedGroupLines)
{
    vector<Line*> Merged_Lines;

    Line* NewLine = new Line();

    int i=0, j=0;


    bool merge_break = false;

    //Finding two Line's in the group that can be merged by Rho
    if(SegmentedGroupLines.size() > 0)
    {
        //For each Line, verify if each next Line can be merged with it
        for(i=0; i<SegmentedGroupLines.size() - 1; i++)
        {
            for(j=i+1; j<SegmentedGroupLines.size(); j++)
            {
                if(CanMergeLines(SegmentedGroupLines.at(i), SegmentedGroupLines.at(j)))
                {
                    //If two Line's can be merged, breaks and calls the MergeSegmentedGroup again, with the new group of Lines.
                    NewLine = FitLineWithMatrix(SegmentedGroupLines.at(i), SegmentedGroupLines.at(j));
                    merge_break = true;
                    break;
                }

            }

            if(merge_break == true)
            {
                break;
            }

        }

    }

    if(merge_break)
    {
        vector<Line*> temp_Lines;

        temp_Lines.push_back(NewLine);

        //Copying the initial Line's to the temporary vector
        for(int ii=0; ii<i; ii++)
        {
            temp_Lines.push_back(SegmentedGroupLines.at(ii));
        }
        for(int jj=i+1; jj<j; jj++)
        {
            temp_Lines.push_back(SegmentedGroupLines.at(jj));
        }
        for(int kk=j+1; kk<SegmentedGroupLines.size(); kk++)
        {
            temp_Lines.push_back(SegmentedGroupLines.at(kk));
        }

        //Recursive call to obtain the final merged Line's
        Merged_Lines = MergeSegmentedGroup(temp_Lines);

    }
    else
    {
        //If no mergeable Line's were found, returns the Group of Lines received
        Merged_Lines = SegmentedGroupLines;
    }

    return Merged_Lines;

}


bool diane_octomap::DianeOctomap::CanMergeLines(Line* LineA, Line* LineB)
{

    bool Result = false;

    double RhoA = LineA->Line_Rho;
    double RhoB = LineB->Line_Rho;


    //If the distance of Rho between the Line's is smaller than a tolerance, it can be merged.
    if((fabs(RhoA - RhoB) <= MergeDeltaRhoTol))
    {
        Result = true;
    }


    return Result;

}


diane_octomap::Line* diane_octomap::DianeOctomap::FitLineWithMatrix(diane_octomap::Line* LineA, diane_octomap::Line* LineB)
{
    //The resultant Line will have the Rho and Theta parameters calculated by the weighted average od the received Line's, where the number of votes were used as the weight
    Line* Result_Line = new Line();

    double Rho = (LineA->Line_Rho * LineA->Line_Votes + LineB->Line_Rho * LineB->Line_Votes)/(LineA->Line_Votes + LineB->Line_Votes);
    double Theta = (LineA->Line_Theta * LineA->Line_Votes + LineB->Line_Theta * LineB->Line_Votes)/(LineA->Line_Votes + LineB->Line_Votes);

    if(Theta > 360)
    {
        Theta = remainder(Theta, 360);
    }

    double Votes = (LineA->Line_Votes + LineB->Line_Votes);

    Result_Line->Line_Rho = Rho;
    Result_Line->Line_Theta = Theta;
    Result_Line->Line_Votes = Votes;


    //Passing the leafs of the old Lines to the new one
    for(int i=0; i<LineA->Leafs_Of_Line.cols(); i++)
    {
        Vector3f leafA = Vector3f(3, 1);
        leafA(0, 0) = LineA->Leafs_Of_Line(0, i);
        leafA(1, 0) = LineA->Leafs_Of_Line(1, i);
        leafA(2, 0) = LineA->Leafs_Of_Line(2, i);

        //If the Matrix is empty, includes the new line
        if(Result_Line->Leafs_Of_Line.cols() == 0)
        {
            int Actual_Size = Result_Line->Leafs_Of_Line.cols();
            Result_Line->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

            //Including the new leaf's informations
            Result_Line->Leafs_Of_Line(0, Actual_Size) = leafA(0, 0);
            Result_Line->Leafs_Of_Line(1, Actual_Size) = leafA(1, 0);
            Result_Line->Leafs_Of_Line(2, Actual_Size) = leafA(2, 0);

        }
        else
        {
            //If it is not, verify if the leaf is not contained in the Matrix.
            if((GroupContainsLeaf(Result_Line->Leafs_Of_Line, leafA)) == false)
            {
                int Actual_Size = Result_Line->Leafs_Of_Line.cols();
                Result_Line->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                //If it is not, includes the new leaf's informations
                Result_Line->Leafs_Of_Line(0, Actual_Size) = leafA(0, 0);
                Result_Line->Leafs_Of_Line(1, Actual_Size) = leafA(1, 0);
                Result_Line->Leafs_Of_Line(2, Actual_Size) = leafA(2, 0);

            }

        }

    }

    for(int j=0; j<LineB->Leafs_Of_Line.cols(); j++)
    {
        Vector3f leafB = Vector3f(3, 1);
        leafB(0, 0) = LineB->Leafs_Of_Line(0, j);
        leafB(1, 0) = LineB->Leafs_Of_Line(1, j);
        leafB(2, 0) = LineB->Leafs_Of_Line(2, j);

        //If the Matrix is empty, includes the new line
        if(Result_Line->Leafs_Of_Line.cols() == 0)
        {
            int Actual_Size = Result_Line->Leafs_Of_Line.cols();
            Result_Line->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

            //Including the new leaf's informations
            Result_Line->Leafs_Of_Line(0, Actual_Size) = leafB(0, 0);
            Result_Line->Leafs_Of_Line(1, Actual_Size) = leafB(1, 0);
            Result_Line->Leafs_Of_Line(2, Actual_Size) = leafB(2, 0);

        }
        else
        {
            //If it is not, verify if the leaf is not contained in the Matrix.
            if((GroupContainsLeaf(Result_Line->Leafs_Of_Line, leafB)) == false)
            {
                int Actual_Size = Result_Line->Leafs_Of_Line.cols();
                Result_Line->Leafs_Of_Line.conservativeResize(3, Actual_Size + 1);

                //If it is not, includes the new leaf's informations
                Result_Line->Leafs_Of_Line(0, Actual_Size) = leafB(0, 0);
                Result_Line->Leafs_Of_Line(1, Actual_Size) = leafB(1, 0);
                Result_Line->Leafs_Of_Line(2, Actual_Size) = leafB(2, 0);

            }

        }

    }


    Result_Line->SortLeafMatrixByX();
    Result_Line->UpdateLimitsWithMatrix();


    return Result_Line;

}


vector<diane_octomap::Line*> diane_octomap::DianeOctomap::SortGroupLines(vector<diane_octomap::Line*> GroupLines)
{
    vector<Line*> SortedGroup;
    vector<Line*> TempGroup;

    int i = 0, j = 0;

    bool swap_break = false;

    for(i=0; i<GroupLines.size() - 1; i++)
    {
        Line* LineA = GroupLines.at(i);

        for(j=i+1; j<GroupLines.size(); j++)
        {
            Line* LineB = GroupLines.at(j);

            if(LineA->Line_Rho > LineB->Line_Rho)
            {
                swap_break = true;
                break;
            }

        }

        if(swap_break)
        {
            break;
        }

    }

    //If an swap happened, calls the sort again, passing the new group (recursive call)
    if(swap_break)
    {
        for(int ii=0; ii<i; ii++)
        {
            TempGroup.push_back(GroupLines.at(ii));
        }

        TempGroup.push_back(GroupLines.at(j));
        TempGroup.push_back(GroupLines.at(i));

        for(int jj=i+1; jj<j; jj++)
        {
            TempGroup.push_back(GroupLines.at(jj));
        }

        for(int kk=j+1; kk<GroupLines.size(); kk++)
        {
            TempGroup.push_back(GroupLines.at(kk));
        }

        SortedGroup = SortGroupLines(TempGroup);

    }
    else
    {
        SortedGroup = GroupLines;
    }

    return SortedGroup;

}


//Looking for a valid sequence of Lines inside the Group of Lines. If a valid sequence is found, the Group becomes a Stair candidate.
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::SequenceFilter(vector<vector<Line *>> lines)
{
    vector<vector<Line*>> FilteredSequence;

    for(int i = 0; i<lines.size();i++)
    {
        if(VerifyLineSequence(lines.at(i)))
        {
            FilteredSequence.push_back(lines.at(i));
        }

    }

    return FilteredSequence;
}


bool diane_octomap::DianeOctomap::VerifyLineSequence(vector<Line*> Group_Lines)
{
    vector<double> Rhos;

    double minZFirstStep = Group_Lines.at(0)->min_Z;

    if(fabs(minZFirstStep) < SequenceZMax)
    {
        for(int j=0; j<Group_Lines.size(); j++)
        {
            Rhos.push_back((Group_Lines.at(j))->Line_Rho);

        }

        for(int k=0; k<Rhos.size()-2; k++)
        {
            for(int l=k+1; l<Rhos.size()-1; l++)
            {

                for(int m=l+1; m<Rhos.size(); m++)
                {
                    double dist1 = fabs(Rhos.at(k) - Rhos.at(l));
                    double dist2 = fabs(Rhos.at(l) - Rhos.at(m));


                    if(((dist1 >= SequenceMinStepWidth) && (dist1 <= SequenceMaxStepWidth)) && ((dist2 >= SequenceMinStepWidth) && (dist2 <= SequenceMaxStepWidth))  )
                    {
                        return true;
                    }

                }

            }

        }
    }

    //If no sequence was found, returns false
    return false;

}


vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::CreateStairCandidatesWithMatrix(vector<vector<diane_octomap::Line*>> Sequenced_Groups)
{
    vector<Stair*> StairCandidates;

    for(int i=0; i<Sequenced_Groups.size(); i++)
    {
        vector<Line*> Grouped_Lines = Sequenced_Groups.at(i);

        Stair* NewStair = new Stair();

        for(int j=0; j<Grouped_Lines.size(); j++)
        {
            Line* line = Grouped_Lines.at(j);

            Step* NewStep = new Step();

            //Passing the Line's leafs to the Step object
            NewStep->Leafs_Of_Step = line->Leafs_Of_Line;

            NewStep->Step_Line = line;

            NewStair->Steps.push_back(NewStep);

        }

        NewStair->Num_Steps = NewStair->Steps.size();

        StairCandidates.push_back(NewStair);

    }

    return StairCandidates;

}


//Updating the stair properties (Executing Least Square for each Step object, to find the optimized line of each Stair)
void diane_octomap::DianeOctomap::UpdateStairProperties(vector<Stair*> StairCandidates)
{
    for(int i=0; i<StairCandidates.size(); i++)
    {
        Stair* stair = StairCandidates.at(i);

        double Sum_Theta = 0.0;
        double Mean_Theta = 0.0;

        //Executing Least Square for each Step object
        for(int j=0; j<stair->Num_Steps; j++)
        {
            Step* step = stair->Steps.at(j);

            step->Step_Line->UpdateLineParametersWithMinSquare();

            Sum_Theta = Sum_Theta + step->Step_Line->Line_Theta;
        }

        //Finding the Mean value of Theta and updating the Steps
        Mean_Theta = Sum_Theta/stair->Num_Steps;

        for(int k=0; k<stair->Num_Steps; k++)
        {
            stair->Steps.at(k)->Step_Line->Line_Theta = Mean_Theta;
        }

    }

}


//Modelling the Stair
vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::ModelStairsWithMatrix(vector<diane_octomap::Stair*>& Stair_Candidates)
{
    vector<Stair*> Modeled_Stairs;

    for(int i=0; i<Stair_Candidates.size(); i++)
    {
        Stair* stair = Stair_Candidates.at(i);

        //Copying to the Stair object, every leafs contained in the Steps objects of this Stair. Calculating minimum and maximum values of (X, Y, Z)
        stair->ExtractLeafsFromStepsMatrix();
        stair->CalculateStairPropertiesWithMatrix();

        //Sorting the Steps of a Stair by the mean value of Z of each Step.
        stair->SortSteps();


        //Modelling a Stair object, using the leaf Matrix
        stair->ModelStair2DWithMatrix(Octree_Resolution);

        //Applying a filter of Stair Defaut Values
        if((stair->Step_Height >= ModellingMinStepHeight) && (stair->Step_Height <= ModellingMaxStepHeight) && (stair->Step_Width >= ModellingMinStepWidth) && (stair->Step_Width <= ModellingMaxStepWidth))
        {
            Modeled_Stairs.push_back(stair);
        }

    }

    return Modeled_Stairs;
}



////Método de escrita de candidato à escada em um arquivo para plot
//void diane_octomap::DianeOctomap::WriteStairCandidateToFileWithMatrix(diane_octomap::Stair* Stair)
//{
//    ofstream stairfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/stair_candidate.txt");

//    stairfile << "A = [";

//    for(int i=0; i<Stair->Leafs_Of_Stair.cols(); i++)
//    {
//        stairfile << Stair->Leafs_Of_Stair(0, i) << ", " << Stair->Leafs_Of_Stair(1, i) << ", " << Stair->Leafs_Of_Stair(2, i) << ";";

//    }





//    stairfile << "];" << endl;
//    stairfile.close();

//}



///Storing Methods for Publishing

void diane_octomap::DianeOctomap::ExtractColumnFilteredPoints(vector<MatrixXf> LeafColumns)
{
    First_Filtered_Points.conservativeResize(3, octree->getNumLeafNodes());

    int First_Filtered_Count = 0;

    for(int i=0; i<LeafColumns.size(); ++i)
    {
        for(int j=0; j<LeafColumns.at(i).cols(); ++j)
        {
            //Adicionando a folha na estrutura referente para o vídeo
            First_Filtered_Points(0, First_Filtered_Count) = (double)LeafColumns.at(i)(0, j);
            First_Filtered_Points(1, First_Filtered_Count) = (double)LeafColumns.at(i)(1, j);
            First_Filtered_Points(2, First_Filtered_Count) = (double)LeafColumns.at(i)(2, j);

            ++First_Filtered_Count;

        }

    }

    First_Filtered_Points.conservativeResize(3, First_Filtered_Count);

}

void diane_octomap::DianeOctomap::ExtractHoughLinesPoints(vector<vector<Line*>> Lines)
{
    for (int i=0; i<Lines.size(); i++)
    {
        for(int j=0; j<Lines.at(i).size(); j++)
        {
            Line* line = Lines.at(i).at(j);

            //Calculando o "a" da reta e um ponto da reta
            float a;

            if(line->Line_Theta > 90 && line->Line_Theta < 270)
            {
                a = tan((270 - line->Line_Theta) * (M_PI/180));
            }
            else
            {
                a = tan((90 + line->Line_Theta) * (M_PI/180));
            }

            float p_x = line->Line_Rho * cos(line->Line_Theta * M_PI/180);
            float p_y = line->Line_Rho * sin(line->Line_Theta * M_PI/180);


            //Calculando os pontos iniciais e finais

            MatrixXf LinePoints = MatrixXf(3, 2);

            LinePoints(0,0) = SpaceXMin;
            LinePoints(1,0) = p_y + a*(SpaceXMin - p_x);
            LinePoints(2,0) = line->Line_Z;

            LinePoints(0,1) = SpaceXMax;
            LinePoints(1,1) = p_y + a*(SpaceXMax - p_x);
            LinePoints(2,1) = line->Line_Z;

            HoughLinesPoints.push_back(LinePoints);

        }

    }

}


void diane_octomap::DianeOctomap::ExtractFilteredHoughLinesPoints(vector<vector<Line*>> Filtered_Groups)
{
    for(int i=0; i<Filtered_Groups.size(); i++)
    {
        for(int j=0; j<Filtered_Groups.at(i).size(); j++)
        {
            Line* line = Filtered_Groups.at(i).at(j);

            //Calculando o "a" da reta e um ponto da reta
            float a;

            if(line->Line_Theta > 90 && line->Line_Theta < 270)
            {
                a = tan((270 - line->Line_Theta) * (M_PI/180));
            }
            else
            {
                a = tan((90 + line->Line_Theta) * (M_PI/180));
            }

            float p_x = line->Line_Rho * cos(line->Line_Theta * M_PI/180);
            float p_y = line->Line_Rho * sin(line->Line_Theta * M_PI/180);


            //Calculando os pontos iniciais e finais

            MatrixXf LinePoints = MatrixXf(3, 2);

            LinePoints(0,0) = SpaceXMin;
            LinePoints(1,0) = p_y + a*(SpaceXMin - p_x);
            LinePoints(2,0) = line->Line_Z;

            LinePoints(0,1) = SpaceXMax;
            LinePoints(1,1) = p_y + a*(SpaceXMax - p_x);
            LinePoints(2,1) = line->Line_Z;

            FilteredHoughLinesPoints.push_back(LinePoints);

        }

    }

}


void diane_octomap::DianeOctomap::ExtractSequencedLinesSegmentsPoints(vector<vector<diane_octomap::Line*>> Filtered_Sequence)
{
    //Obtaining the sequenced Line segment's points for plotting
    for(int m=0; m<Filtered_Sequence.size(); m++)
    {
        for(int n=0; n<Filtered_Sequence.at(m).size(); n++)
        {
            Line* line = Filtered_Sequence.at(m).at(n);

            line->SortLeafMatrixByX();

            //Evaluating the line's "a" value and a finding a point in the line
            float a;

            if(line->Line_Theta > 90 && line->Line_Theta < 270)
            {
                a = tan((270 - line->Line_Theta) * (M_PI/180));
            }
            else
            {
                a = tan((90 + line->Line_Theta) * (M_PI/180));
            }

            float p_x = line->Line_Rho * cos(line->Line_Theta * M_PI/180);
            float p_y = line->Line_Rho * sin(line->Line_Theta * M_PI/180);


            //Selecting the minimum and maximum values of X-coordinate in the Line
            float Min_X, Max_X;

            Min_X = line->Leafs_Of_Line(0, 0);
            Max_X = line->Leafs_Of_Line(0, line->Leafs_Of_Line.cols() - 1);


            //Finding the inicial and final points in the line
            if(fabs((line->max_Z - line->min_Z)) < 0.40)
            {
                for(int o=0; o<=round(fabs((line->max_Z - line->min_Z))/0.05); ++o)
                {
                    MatrixXf LinePoints = MatrixXf(3, 2);

                    LinePoints(0,0) = Min_X;
                    LinePoints(1,0) = p_y + a*(Min_X - p_x);
                    LinePoints(2,0) = line->min_Z + 0.05*o;

                    LinePoints(0,1) = Max_X;
                    LinePoints(1,1) = p_y + a*(Max_X - p_x);
                    LinePoints(2,1) = line->min_Z + 0.05*o;

                    SequencedLinesSegmentsPoints.push_back(LinePoints);

                }

            }
            else
            {
                for(int p=0; p<4; ++p)
                {
                    MatrixXf LinePoints = MatrixXf(3, 2);

                    LinePoints(0,0) = Min_X;
                    LinePoints(1,0) = p_y + a*(Min_X - p_x);
                    LinePoints(2,0) = line->max_Z - 0.05*p;

                    LinePoints(0,1) = Max_X;
                    LinePoints(1,1) = p_y + a*(Max_X - p_x);
                    LinePoints(2,1) = line->max_Z - 0.05*p;

                    SequencedLinesSegmentsPoints.push_back(LinePoints);

                }

            }

        }

    }

}




//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------




//void diane_octomap::DianeOctomap::StairDetection()
//{
//    //Início da marcacão de tempo de execucao da funcão
//    time_t t_begin, t_end;

//    t_begin = time(0);


//    //***Obtendo os mínimos e os máximos (para obter o comprimento, a largura e a altura do espaco) --- Para Utilizar na Transformada de Hough
//    vector<double> Space_Properties = GetSpaceProperties();



//    //***Realizando a transformada de Hough e aplicando um filtro para obter somente os planos que receberam uma quantidade de votos dentro de um intervalo limite***
//    //***O intervalo limite deve ser apdatativo (caso a resolucão do octomap mude, não saberemos qual será o intervalo limite correto)***
//    vector<vector<double>> Hough_Planes = PlanesHoughTransform(Space_Properties.at(0), Space_Properties.at(1), Space_Properties.at(2));


//    //Escrevendo nos arquivos os planos resultantes da Transformada de Hough (para plot no Matlab) --- Retirar para que diminuir o processamento;
//    WriteFilteredPlanesToFile(Hough_Planes);



//    //***Aglutinando (Merge) nos planos que são praticamente coplanares (inicialmente só utilizamos merge para planos que possuem o mesmo theta e o mesmo phi)***
//    //***Assim, o merge foi aplicado para planos que estavam muito próximos entre si***

//    //Chamando o método recursivo que aglutina os planos aproximadamente coplanares
//    vector<vector<double>> Merged_Planes = MergePlanes(Hough_Planes);


//    cout << "Após o merge, existem " << Merged_Planes.size() << " planos." << endl << endl;


//    //Escrevendo os planos aglutinados em um arquivo (para plot no Matlab);
//    WriteMergedPlanesToFile(Merged_Planes);



//    //***Agrupando os planos restantes por seu Theta e Phi***
//    //***Conferir se desse modo, o Rho poderia ser identificado como a largura do degrau***
//    vector<vector<vector<double>>> Grouped_Planes = GroupPlanesByThetaPhi(Merged_Planes);



//    //***Utilizando um histograma como um "Filtro" e para identificar a distância entre planos que mais ocorre***
//    //***Foi inserido um filtro nesse histograma para só pontuar distâncias que estejam dentro do padrão de escadas***
//    vector<vector<vector<double>>> Histogram_Grouped_Planes = GenerateHistogram(Grouped_Planes);


//    //***Para cada grupo, verifica se existe uma sequência de 3 planos que estejam com distâncias dentro dos limites encontrados no histograma***
//    vector<vector<vector<double>>> Sequenced_Grouped_Planes = FilterGroups(Histogram_Grouped_Planes);



//    //***Estruturando uma escada para cada grupo de planos ainda existente***
//    //***Dado que buscamos os planos verticais da escada, cada plano deveria nos retornar um degrau***
//    //Obtendo os candidatos à escada
//    vector<Stair*> StairCandidatesDetected = StairCandidatesDetection(Sequenced_Grouped_Planes);

////    WriteStairCandidateToFile(StairCandidatesDetected.at(9));


//    //***Retirando as folhas dos degraus de cada candidato à escada que estão fora do padrão de degraus
//    CleanStairsLeafs(StairCandidatesDetected);





//    //***Atualizando os candidatos à escada
//    //Para cada escada, retira os degraus que não são válidos. Se o candidato à escada não possuir mais do que 3 degraus válidos, ele não será mais um candidato.
//    vector<Stair*> NewStairCandidates = CleanStairSteps(StairCandidatesDetected);


//    //Escrevendo as informacões das escadas em um arquivo (para plot no Matlab)
////    WriteStairCandidateToFile(NewStairCandidates.at(0));


//    //***Modelando cada candidato à escada (obtendo o comprimento, largura e altura média dos degraus, a aresta inicial e os pontos de arestas referentes aos outros degraus)
//    vector<Stair*> Modeled_Stairs = ModelStairs(NewStairCandidates);



//    //Escrevendo em um arquivo a aresta, comprimento, largura e altura da 1a escada (para plot no Matlab)
////    WriteModeledStairPropertiesToFile(Modeled_Stairs.at(0));





//    //Fim da marcacao de tempo de execucao
//    t_end = time(0);
//    time_t tempo = t_end - t_begin;

//    cout << "Demorou " << tempo << " segundos." << endl << endl;

//}



//vector<double> diane_octomap::DianeOctomap::GetSpaceProperties()
//{
//    vector<double> Space_Properties;

//    double min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;

//    for(int i = 0; i < OccupiedLeafsInBBX.size(); i++)
//    {
//        OcTree::leaf_bbx_iterator leaf = OccupiedLeafsInBBX.at(i);

//        if(leaf.getX() < min_x)
//        {
//            min_x = leaf.getX();
//        }
//        if(leaf.getX() > max_x)
//        {
//            max_x = leaf.getX();
//        }

//        if(leaf.getY() < min_y)
//        {
//            min_y = leaf.getY();
//        }
//        if(leaf.getY() > max_y)
//        {
//            max_y = leaf.getY();
//        }

//        if(leaf.getZ() < min_z)
//        {
//            min_z = leaf.getZ();
//        }
//        if(leaf.getZ() > max_z)
//        {
//            max_z = leaf.getZ();
//        }
//    }

//    double Space_Length = max_x - min_x;
//    double Space_Width = max_y - min_y;
//    double Space_Height = max_z - min_z;

//    Space_Properties.push_back(Space_Length);
//    Space_Properties.push_back(Space_Width);
//    Space_Properties.push_back(Space_Height);

//    return Space_Properties;

//}




////Os dados dos centróides serão obtidos diretamente do vetor de folhas
//vector<vector<double>> diane_octomap::DianeOctomap::PlanesHoughTransform(double length, double width, double height)
//{
//    cout << "Funcão PlanesHoughTransform inicializada." << endl << endl;

//    //Definindo o Rho, Theta e Phi mínimos e máximos (dado que só estamos querendo detectar planos verticais, estabelecemos um phi mínimo e um máximo para reduzir o processamento)
//    Rho_Min = 0;
//    Rho_Max = ceil(sqrt(pow(length, 2) + pow(width, 2) + pow(height, 2)));
//    Theta_Min = 0;
//    Theta_Max = 360;
//    Phi_Min = 70;
//    Phi_Max = 110;

//    Rho_Passo = 0.05;
//    Theta_Passo = 5;
//    Phi_Passo = 5;


//    Rho_Num = (Rho_Max - Rho_Min)/Rho_Passo;
//    Theta_Num = (Theta_Max - Theta_Min)/Theta_Passo;
//    Phi_Num = (Phi_Max - Phi_Min)/Phi_Passo;


//    cout << "(Rho_Min, Rho_Max, Theta_Min, Theta_Max, Phi_Min, Phi_Max): (" << Rho_Min << ", " << Rho_Max << ", " << Theta_Min << ", " << Theta_Max << ", " << Phi_Min << ", " << Phi_Max << ")" << endl;
//    cout << "Rho_Num: " << Rho_Num << ", Theta_Num: " << Theta_Num << ", Phi_Num: " << Phi_Num << "." << endl;


//    //Inicializando o Acumulador
//    InitializeAccumulator();


//    //Para cada voxel ocupada, chama a funcão de acumulacão para realizar a votacão
//    for(int i = 0; i<OccupiedLeafsInBBX.size(); i++)
//    {
//        AccumulatePoint(OccupiedLeafsInBBX.at(i));

//    }

//    cout << endl << "Votacão finalizada" << endl;


//    //Printando o estado do acumulador
////    PrintAccumulator();


//    //Obtendo os planos que passaram pelo filtro
//    vector<vector<double>> Filtered_Planes = GetFilteredPlanes();


//    cout << "Existem " << Filtered_Planes.size() << " planos filtrados." << endl << endl;


//    cout << "Funcão PlanesHoughTransform finalizada." << endl;


//    return Filtered_Planes;


//}


//void diane_octomap::DianeOctomap::InitializeAccumulator()
//{
//    //Initializing the acumulator with 0's;
//    for(int i=0; i < Rho_Num; i++)
//    {
//        vector<vector<int>> vector_thetas;

//        for(int j=0; j < Theta_Num; j++)
//        {
//            vector<int> vector_phis;

//            for(int k=0; k < Phi_Num; k++)
//            {
//                vector_phis.push_back(0);
//            }

//            vector_thetas.push_back(vector_phis);
//        }

//        Accumulator.push_back(vector_thetas);
//    }

//}


//void diane_octomap::DianeOctomap::AccumulatePoint(OcTree::leaf_bbx_iterator leaf_point)
//{
//    //Invertendo a ordem da iteracão para diminuir a quantidade de cálculos
//    for(unsigned int i = 0; i<Phi_Num; i++)
//    {
//        double phi = (Phi_Min + (i+0.5)*Phi_Passo)*(M_PI/180);


//        //Garantindo que o phi não ultrapassará o seu limite máximo
//        if(phi > (Phi_Max*M_PI/180))
//        {
//            phi = (Phi_Max*M_PI/180);
//        }

//        for(unsigned int j = 0; j<Theta_Num; j++)
//        {
//            double theta = (Theta_Min + (j+0.5)*Theta_Passo)*(M_PI/180);


//            //Garantindo que o theta não ultrapassará o seu limite máximo
//            if(theta > ((Theta_Max*M_PI)/180))
//            {
//                theta = ((Theta_Max*M_PI)/180);
//            }

//            //Definindo a normal do plano
//            double n[3];
//            n[0] = cos(theta)*sin(phi);
//            n[1] = sin(theta)*sin(phi);
//            n[2] = cos(phi);

//            for(unsigned int k = 0; k<Rho_Num; k++)
//            {
//                double rho = (Rho_Min + (k+0.5)*Rho_Passo);

//                //Calculando o Rho do plano que passa no ponto sendo avaliado com o Theta e o Phi avaliados;
//                double point_rho = leaf_point.getX() * n[0] + leaf_point.getY() * n[1] + leaf_point.getZ() * n[2];

//                //Se a distância entre os dois planos, que possuem a mesma angulacão theta e phi, estiver dentro da tolerância, o ponto vota nessa célula
//                if (fabs(point_rho - rho) < Max_Planes_Distance)
//                {
//                    ((Accumulator.at(k)).at(j)).at(i) = ((Accumulator.at(k)).at(j)).at(i) + 1;
//                }

//            }

//        }

//    }

//}


//vector<vector<double>> diane_octomap::DianeOctomap::GetFilteredPlanes()
//{
//    //Armazenando no vetor as coordenadas polares do centróide da célula
//    vector<vector<double>> Filtered_Planes;

//    int count = 0;

//    for(unsigned int i = 0; i<Phi_Num; i++)
//    {
//        double phi = (Phi_Min + (i+0.5)*Phi_Passo);

//        if ((phi>= Filter_Phi_Min) && (phi<= Filter_Phi_Max))
//        {
//            for(unsigned int j = 0; j<Theta_Num; j++)
//            {
//                double theta = (Theta_Min + (j+0.5)*Theta_Passo);

//                for(unsigned int k = 0; k<Rho_Num; k++)
//                {
//                    double rho = (Rho_Min + (k+0.5)*Rho_Passo);

//                    double votes = ((Accumulator.at(k)).at(j)).at(i);

//                    if ((votes >= Filter_Vote_Min) && (votes <= Filter_Vote_Max))
//                    {
//                        //Inclui no vetor filtrado
//                        vector<double> valid_plane;
//                        valid_plane.push_back(rho);
//                        valid_plane.push_back(theta);
//                        valid_plane.push_back(phi);
//                        valid_plane.push_back(votes);

//                        Filtered_Planes.push_back(valid_plane);
//                        count++;
//                    }

//                }

//            }

//        }

//    }

//    cout << count << " planos passaram pelo filtro." << endl << endl;

//    return Filtered_Planes;

//}


//vector<vector<double>> diane_octomap::DianeOctomap::MergePlanes(vector<vector<double>> Planes)
//{
//    //Input:
//    //Planes é o vector com todos os vetores referentes aos Planos a serem verificados para aglutinacão.
//    //Cada vector em Planes contém: RHO, THETA, PHI e Votos nesse plano

//    //Output:
//    //Merged_Planes é o vector resultado com todos os vetores de Planos após serem aglutinados.

//    vector<vector<double>> Merged_Planes;

//    vector<double> NewPlane;

//    int i = 0, j = 0;


//    bool merge_break = false;

////    cout << "Tamanho de Planes: " << Planes.size() << "." << endl;

//    if(Planes.size() > 0)
//    {
//        //Para cada plano, verifica se cada um dos planos seguintes podem ser agrupados com o inicial
//        for(i=0; i<Planes.size(); i++)
//        {
//            for(j=i+1; j<Planes.size(); j++)
//            {
//                if(CanMergePlanes(Planes.at(i), Planes.at(j)))
//                {
//                    //Se dois planos forem marcados como mergeable, executa o MergePlanes repassando o novo Planes (com o novo Plane e sem os dois antigos)
//                    NewPlane = FitPlane(Planes.at(i), Planes.at(j));
//                    merge_break = true;
//                    break;
//                }

//            }

//            if (merge_break == true)
//            {
//                break;
//            }

//        }

//    }

//    //Se ocorreu um merge_break, gera um novo conjunto de Planes (com o novo Plane e sem os dois antigos) e chama o MergePlanes
//    if(merge_break)
//    {
//        vector<vector<double>> temp_Planes;

//        temp_Planes.push_back(NewPlane);

//        //Copiando os planos antigos para o novo vetor
//        for(int ii=0; ii<i; ii++)
//        {
//            temp_Planes.push_back(Planes.at(ii));
//        }
//        for(int jj = i+1; jj<j; jj++)
//        {
//            temp_Planes.push_back(Planes.at(jj));
//        }
//        for(int kk = j+1; kk<Planes.size(); kk++)
//        {
//            temp_Planes.push_back(Planes.at(kk));
//        }

////        cout << "Tamanho de temp_Planes: " << temp_Planes.size() << "." << endl;

//        //Recursão para obter somente os planos aglutinados finais
//        Merged_Planes = MergePlanes(temp_Planes);

//    }
//    else
//    {
//        //Se não existirem mais planos a serem aglutinados, retorna o vector que foi recebido
//        Merged_Planes = Planes;
//    }

//    return Merged_Planes;

//}


//bool diane_octomap::DianeOctomap::CanMergePlanes(vector<double> PlaneA, vector<double> PlaneB)
//{
//    //Input:
//    //PlaneA é o vector com informacões do primeiro plano sendo analisado, contendo RHO, THETA e PHI.
//    //PlaneB é o vector com informacões do segundo plano sendo analisado, contendo RHO, THETA e PHI.

//    //Output:
//    //Bool Result indicando se os dois planos podem ser aglutinados ou não

//    //delta_Rho é a tolerância em Rho para a aglutinacão entre os planos.
//    //delta_Theta é a tolerância em Theta para a aglutinacão entre os planos.
//    //delta_Phi é a tolerância em Phi para a aglutinacão entre os planos.


//    bool Result = false;

//    double RhoA = PlaneA.at(0);
//    double RhoB = PlaneB.at(0);
//    double ThetaA = PlaneA.at(1);
//    double ThetaB = PlaneB.at(1);
//    double PhiA = PlaneA.at(2);
//    double PhiB = PlaneB.at(2);

//    if ((abs(RhoA - RhoB) <= delta_Rho) && (abs(ThetaA - ThetaB) <= delta_Theta) && (abs(PhiA - PhiB) <= delta_Phi))
//    {
//        //Seria bom adicionar algum outro filtro fora os limites? Não tenho a informacão dos pontos presentes no plano
//        Result = true;
//    }

//    return Result;

//}


//vector<double> diane_octomap::DianeOctomap::FitPlane(vector<double> PlaneA, vector<double> PlaneB)
//{
//    //Inicialmente, o plano resultante possuirá parâmetros RHO, THETA e PHI iguais às médias ponderadas dos parâmetros dos planos recebidos, onde o peso é a quantidade de votos.

//    vector<double> Result_Plane;

//    double Rho = ((PlaneA.at(0)*PlaneA.at(3) + PlaneB.at(0)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
//    double Theta = ((PlaneA.at(1)*PlaneA.at(3) + PlaneB.at(1)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
//    double Phi = ((PlaneA.at(2)*PlaneA.at(3) + PlaneB.at(2)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
//    double Votes = (PlaneA.at(3) + PlaneB.at(3));



//    Result_Plane.push_back(Rho);
//    Result_Plane.push_back(Theta);
//    Result_Plane.push_back(Phi);
//    Result_Plane.push_back(Votes);


//    return Result_Plane;
//}


////Separando os planos em grupos de par (Theta, Phi)
//vector<vector<vector<double>>> diane_octomap::DianeOctomap::GroupPlanesByThetaPhi(vector<vector<double>> Planes)
//{
//    vector<vector<vector<double>>> GroupedPlanes;

//    //Separando os planos por seus ângulos Theta e Phi
//    for(int i=0; i<Planes.size(); i++)
//    {
//        vector<double> Plane = Planes.at(i);

//        double theta = Plane.at(1);
//        double phi = Plane.at(2);

//        bool Group_Found = false;

//        //Verificando se os ângulos do Plano já fazem parte de um grupo
//        for(int j=0; j<GroupedPlanes.size(); j++)
//        {
//            if((theta == ((GroupedPlanes.at(j)).at(0)).at(1)) && (phi == ((GroupedPlanes.at(j)).at(0)).at(2)))
//            {
//                GroupedPlanes.at(j).push_back(Plane);
//                Group_Found = true;
//                break;
//            }

//        }

//        if(Group_Found == false)
//        {
//            //Se o grupo não foi encontrado, cria um novo grupo
//            vector<vector<double>> NewGroupPlanes;
//            NewGroupPlanes.push_back(Plane);

//            GroupedPlanes.push_back(NewGroupPlanes);
//        }

//    }

//    return GroupedPlanes;

//}


//vector<vector<vector<double>>> diane_octomap::DianeOctomap::GenerateHistogram(vector<vector<vector<double>>> Grouped_Planes)
//{
//    //Montagem do histograma de frequência de distância entre os planos em um grupo
//    double Hist_Passo = 0.025;
//    int Hist_Length = ceil((Rho_Max - Rho_Min))/Hist_Passo; //Quantidade de intervalos do histograma
//    vector<int> Histogram(Hist_Length, 0);

//    map<vector<vector<double>>, double> Planes_Dist_Map;

//    //Para cada grupo de planos (theta e phi fixos), verifica as distâncias entre os planos e pontua no histograma
//    for(int k=0; k<Grouped_Planes.size(); k++)
//    {
//        if((Grouped_Planes.at(k)).size() >= Min_Num_Steps)
//        {
//            for(int l=0; l<(Grouped_Planes.at(k)).size() - 1; l++)
//            {
//                vector<double> PlaneA = (Grouped_Planes.at(k)).at(l);
//                vector<double> PlaneB;

//                for(int m=l+1; m<(Grouped_Planes.at(k)).size(); m++)
//                {
//                    PlaneB = (Grouped_Planes.at(k)).at(m);

//                    double Dist = abs(PlaneA.at(0) - PlaneB.at(0));

//                    //Só pontua se a distância estiver dentro do padrão de degrau
//                    if((Dist > Min_Step_Width) && (Dist < Max_Step_Width))
//                    {
//                        //Adicionando um ponto para o intervalo do histograma.
//                        int index = floor(Dist/Hist_Passo);

//                        Histogram.at(index) = Histogram.at(index) + 1;

//                        //Adicionar em alguma estrutura os dois planos que geraram essa distância
//                        vector<vector<double>> Planes_Key;
//                        Planes_Key.push_back(PlaneA);
//                        Planes_Key.push_back(PlaneB);

//                        Planes_Dist_Map.insert(pair< vector<vector<double>>, double>(Planes_Key, Dist));

//                    }

//                }

//            }

//        }

//    }

//    int max_freq = 0;
//    int max_index = 0;
//    for (int n=0; n<Hist_Length; n++)
//    {
//        if (Histogram.at(n) > max_freq)
//        {
//            max_freq = Histogram.at(n);
//            max_index = n;
//        }
//    }

//    //Obtendo o intervalo de distância tolerado para selecionar os planos
//    Histogram_Dist_Min = (max_index - 1) * Hist_Passo;
//    Histogram_Dist_Max = (max_index + 2) * Hist_Passo;


//    cout << "Maior pontuacão do histograma: " << max_freq << "." << endl << endl;
//    cout << "Índice da maior pontuacão do histograma: " << max_index << "." << endl << endl;

//    cout << "Distância mínima: " << Histogram_Dist_Min << "." << endl << endl;
//    cout << "Distância máxima: " << Histogram_Dist_Max << "." << endl << endl;


//    //Extraindo do map os planos que fazem parte de um par de planos com distância tolerada
//    return ExtractValidPlanes(Planes_Dist_Map);

//}


//vector<vector<vector<double>>> diane_octomap::DianeOctomap::ExtractValidPlanes(map<vector<vector<double>>, double> Planes_Dist_Map)
//{
//    //Verificando se o par de planos possuem uma distância dentro da tolerância
//    vector<vector<double>> Valid_Planes;

//    vector<vector<vector<double>>> Grouped_Valid_Planes;


//    for(map<vector<vector<double>>, double>::iterator it=Planes_Dist_Map.begin(); it!=Planes_Dist_Map.end(); it++)
//    {
//        if((it->second >= Histogram_Dist_Min) && (it->second <= Histogram_Dist_Max))
//        {
//            vector<vector<double>> Plane_Pair = it->first;

//            //Inserindo em Valid_Planes o 1o plano se já não existir
//            if(!(find(Valid_Planes.begin(), Valid_Planes.end(), Plane_Pair.at(0)) != Valid_Planes.end()))
//            {
//                Valid_Planes.push_back(Plane_Pair.at(0));
//            }

//            //Inserindo em Valid_Planes o 2o plano se já não existir
//            if(!(find(Valid_Planes.begin(), Valid_Planes.end(), Plane_Pair.at(1)) != Valid_Planes.end()))
//            {
//                Valid_Planes.push_back(Plane_Pair.at(1));
//            }

//        }

//    }

//    cout << "Existem " << Valid_Planes.size() << " planos válidos." << endl << endl;


//    //Agrupando novamente por Theta e Phi
//    Grouped_Valid_Planes = GroupPlanesByThetaPhi(Valid_Planes);


//    cout << "Existem " << Grouped_Valid_Planes.size() << " grupos de planos válidos." << endl << endl;


//    return Grouped_Valid_Planes;

//}


//vector<vector<vector<double>>> diane_octomap::DianeOctomap::FilterGroups(vector<vector<vector<double>>> Groups_Planes)
//{
//    vector<vector<vector<double>>> Sequenced_Groups;

//    //Filtrando os grupos de planos (se ele não possuir uma sequência, o grupo inteiro é removido)
//    for(int i=0; i<Groups_Planes.size(); i++)
//    {
//        vector<vector<double>> Group = Groups_Planes.at(i);

//        if(VerifySequence(Group))
//        {
//            Sequenced_Groups.push_back(Group);
//        }

//    }

//    return Sequenced_Groups;
//}


//bool diane_octomap::DianeOctomap::VerifySequence(vector<vector<double>> Group_Planes)
//{
//    vector<double> Rhos;

//    for(int j=0; j<Group_Planes.size(); j++)
//    {
//        Rhos.push_back((Group_Planes.at(j)).at(0));
//    }


//    sort(Rhos.begin(), Rhos.end());

//    for(int k=0; k<Rhos.size()-2; k++)
//    {
//        for(int l=k+1; l<Rhos.size()-1; l++)
//        {

//            for(int m=l+1; m<Rhos.size(); m++)
//            {
//                double dist1 = fabs(Rhos.at(k) - Rhos.at(l));
//                double dist2 = fabs(Rhos.at(l) - Rhos.at(m));

//                if(((dist1 >= Histogram_Dist_Min) && (dist1 <= Histogram_Dist_Max)) && ((dist2 >= Histogram_Dist_Min) && (dist2 <= Histogram_Dist_Max)) )
//                {
//                    return true;
//                }

//            }

//        }

//    }

//    //Se não encontrou uma sequencia, retorna false
//    return false;

//}


//vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::StairCandidatesDetection(vector<vector<vector<double>>> Grouped_Planes)
//{
//    vector<Stair*> StairCandidatesDetected;

//    //Obtendo os candidatos à escada
//    for(int i=0; i<Grouped_Planes.size(); i++)
//    {
//        //Se no grupo de planos existirem planos suficientes para detectar uma escada, obtém a candidata da escada
//        if(Grouped_Planes.at(i).size() >= Min_Num_Steps)
//        {
//            Stair* NewStairCandidate = ObtainStairCandidateFromGroup(Grouped_Planes.at(i));

//            if((NewStairCandidate->Steps.size() != 0))
//            {
//                StairCandidatesDetected.push_back(NewStairCandidate);

//            }

//        }

//    }

//    return StairCandidatesDetected;

//}



//diane_octomap::Stair* diane_octomap::DianeOctomap::ObtainStairCandidateFromGroup(vector<vector<double>> Group_Planes)
//{
//    Stair* StairCandidate = new Stair();

//    //Para cada plano do grupo, armazena os pontos que fazem parte/estão à uma distância tolerável deste plano
//    for(int i=0; i<Group_Planes.size(); i++)
//    {
//        Step* NewStep = new Step();

//        vector<double> Plane = Group_Planes.at(i);
//        double rho = Plane.at(0);
//        double theta = Plane.at(1) * (M_PI/180);
//        double phi = Plane.at(2) * (M_PI/180);

//        //Definindo a normal do plano
//        double n[3];
//        n[0] = cos(theta)*sin(phi);
//        n[1] = sin(theta)*sin(phi);
//        n[2] = cos(phi);

//        NewStep->Step_Plane = Plane;

//        //Para cada folha, verifica se a centróide está próxima do plano
//        for(int j=0; j<OccupiedLeafsInBBX.size(); j++)
//        {
//            OcTree::leaf_bbx_iterator leaf = OccupiedLeafsInBBX.at(j);

//            //Calculando o Rho do plano que passa no ponto sendo avaliado com o Theta e o Phi do plano avaliado;
//            double point_rho = leaf.getX() * n[0] + leaf.getY() * n[1] + leaf.getZ() * n[2];

//            //Se a distância entre os dois planos, que possuem a mesma angulacão theta e phi, estiver dentro da tolerância, o ponto vota nessa célula
//            if (fabs(point_rho - rho) < Max_Planes_Distance)
//            {
//                NewStep->Leafs_In_Step.push_back(leaf);
//            }

//        }

//        //Inserindo o Degrau referente ao plano no candidato à escada
//        StairCandidate->Steps.push_back(NewStep);

//    }

//    return StairCandidate;

//}


//void diane_octomap::DianeOctomap::CleanStairsLeafs(vector<Stair*>& Detected_Stair_Candidates)
//{
//    for(int i=0; i<Detected_Stair_Candidates.size(); i++)
//    {
//        Stair* stair = Detected_Stair_Candidates.at(i);

//        for(int j=0; j<stair->Steps.size(); j++)
//        {
//            Step* step = stair->Steps.at(j);

//            //Para cada degrau, filtra as colunas que possuem alturas fora do padrão de degraus
//            step->StepHeightFilter();

//            //Aplicando o filtro de histograma para o degrau (Degraus que não tiverem um padrão nas alturas do seu degrau serão considerados inválidos)
//            step->StepHeightHistogram();

//        }

//    }

//}


//vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::CleanStairSteps(vector<diane_octomap::Stair*>& Detected_Stair_Candidates)
//{
//    vector<Stair*> NewStairCandidates;

//    for(int i=0; i<Detected_Stair_Candidates.size(); i++)
//    {
//        Stair* stair = Detected_Stair_Candidates.at(i);

//        vector<Step*> NewSteps;
//        NewSteps.clear();

//        for(int j=0; j<stair->Steps.size(); j++)
//        {
//            Step* step = stair->Steps.at(j);

//            //Verifica se o degrau possui folhas (se não tiver, não é um degrau válido)
//            if(step->Leafs_In_Step.size() > 0)
//            {
//                NewSteps.push_back(step);
//            }
//        }

//        //Se a quantidade de degraus válidos for o suficiente para formar uma escada, essa escada ainda é uma candidata
//        if(NewSteps.size() > Min_Num_Steps)
//        {
//            stair->Steps = NewSteps;
//            stair->Num_Steps = NewSteps.size();

//            NewStairCandidates.push_back(stair);
//        }

//    }

//    return NewStairCandidates;

//}


//vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::ModelStairs(vector<diane_octomap::Stair*>& Stair_Candidates)
//{
//    vector<Stair*> Modeled_Stairs;

//    for(int i=0; i<Stair_Candidates.size(); i++)
//    {
//        Stair* stair = Stair_Candidates.at(i);

//        //Passando para a escada todos as folhas presentes nos seus degraus e calculando os X, Y e Z mínimos e máximos
//        stair->ExtractLeafsFromSteps();
//        stair->CalculateStairProperties();


//        //Ordenando os degraus da escada de acordo com a posicão média Z de cada degrau, em ordem crescente
//        stair->SortSteps();


//        //Modelando a escada
//        //stair->ModelStair(Octree_Resolution);
//        stair->ModelStair2D(Octree_Resolution);


//        Modeled_Stairs.push_back(stair);

//    }

//    return Modeled_Stairs;

//}



//Método printando os votos do Acumulador
//void diane_octomap::DianeOctomap::PrintAccumulator()
//{
//    int count = 0;

//    for(unsigned int i = 0; i<Phi_Num; i++)
//    {
//        double phi1 = (Phi_Min + (i)*Phi_Passo);
//        double phi2 = (Phi_Min + (i+1)*Phi_Passo);

//        if ((phi1 >= Filter_Phi_Min) && (phi2 <= Filter_Phi_Max))
//        {
//            for(unsigned int j = 0; j<Theta_Num; j++)
//            {
//                double theta1 = (Theta_Min + (j)*Theta_Passo);
//                double theta2 = (Theta_Min + (j+1)*Theta_Passo);

//                for(unsigned int k = 0; k<Rho_Num; k++)
//                {
//                    double rho1 = (Rho_Min + (k)*Rho_Passo);
//                    double rho2 = (Rho_Min + (k+1)*Rho_Passo);

//                    double votes = 0;

//                    votes = ((Accumulator.at(k)).at(j)).at(i);

//                    if ((votes > Filter_Vote_Min) && (votes < Filter_Vote_Max))
//                    {

//                        cout << "(rho1, rho2, theta1, theta2, phi1, phi2, Total Votes): (" << rho1 << ", " << rho2 << ", " << theta1 << ", " << theta2 << ", " << phi1 << ", " << phi2 << ", " << votes << ")" << endl;
//                        cout << endl;

//                        count++;

//                    }

//                }

//            }

//            cout << endl << endl;

//        }

//    }

//    cout << "Quantidade de planos encontrados: " << count << "." << endl;

//}


//void diane_octomap::DianeOctomap::WriteFilteredPlanesToFile(vector<vector<double>> Filtered_Planes)
//{
//    ofstream filteredfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/filtered_planes.txt");

//    filteredfile << "A = [";

//    for(int i=0; i<Filtered_Planes.size(); i++)
//    {
//        vector<double> Filtered_Plane = Filtered_Planes.at(i);

//        filteredfile << Filtered_Plane.at(0) << " " << Filtered_Plane.at(1) << " " << Filtered_Plane.at(2) << ";";
//    }


//    filteredfile << "]" << endl;
//    filteredfile.close();

//}


//void diane_octomap::DianeOctomap::WriteMergedPlanesToFile(vector<vector<double>> Merged_Planes)
//{
//    ofstream mergedfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/merged_planes.txt");

//    mergedfile << "A = [";

//    for(int i=0; i<Merged_Planes.size(); i++)
//    {
//        vector<double> Merged_Plane = Merged_Planes.at(i);

//        mergedfile << Merged_Plane.at(0) << " " << Merged_Plane.at(1) << " " << Merged_Plane.at(2) << ";";
//    }


//    mergedfile << "]" << endl;
//    mergedfile.close();

//}

//void diane_octomap::DianeOctomap::WriteStairCandidateToFile(diane_octomap::Stair* Stair)
//{
//    ofstream stairfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/stair_candidate.txt");

//    stairfile << "A = [";

//    for(int i=0; i<Stair->Steps.size(); i++)
//    {
//        Step* step = Stair->Steps.at(i);

//        for(int j=0; j<step->Leafs_In_Step.size(); j++)
//        {
//            OcTree::leaf_bbx_iterator leaf = step->Leafs_In_Step.at(j);

//            stairfile << leaf.getX() << ", " << leaf.getY() << ", " << leaf.getZ() << ";";
//        }

//    }


//    stairfile << "];" << endl;
//    stairfile.close();

//}


///*** Lógica utilizando planos ***





void diane_octomap::DianeOctomap::WriteModeledStairPropertiesToFile(diane_octomap::Stair* Stair)
{
    ofstream stairfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/modeled_stair_properties.txt");

    stairfile << "PointA = [";

    stairfile << Stair->Aresta.at(0).at(0) << ", " << Stair->Aresta.at(0).at(1) << ", " << Stair->Aresta.at(0).at(2) << "];" << endl << endl;


    stairfile << "PointB = [";

    stairfile << Stair->Aresta.at(1).at(0) << ", " << Stair->Aresta.at(1).at(1) << ", " << Stair->Aresta.at(1).at(2) << "];" << endl << endl;


    stairfile << "Num_Steps = " << Stair->Num_Steps << ";" << endl << endl;

    stairfile << "Step_Length = " << Stair->Step_Length << ";" << endl << endl;

    stairfile << "Step_Width = " << Stair->Step_Width << ";" << endl << endl;

    stairfile << "Step_Height = " << Stair->Step_Height << ";" << endl << endl;


    stairfile.close();

}


diane_octomap::DianeOctomap::~DianeOctomap()
{
    StopInternalCycle();
}



///******Métodos da classe Step*******

diane_octomap::Step::Step()
{
    Step_Min_Z = 1000;
    Step_Max_Z = -1000;
    Step_Mean_Z = 0;

    Min_Step_Height = 0.09;
    Max_Step_Height = 0.20;

}


void diane_octomap::Step::CalculateStepProperties()
{
    //Obtendo o menor Z e o maior Z das folhas contidas no degrau
    for(int i=0; i<Leafs_In_Step.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = Leafs_In_Step.at(i);

        if(Step_Min_Z > leaf.getZ())
        {
            Step_Min_Z = leaf.getZ();
        }
        if(Step_Max_Z < leaf.getZ())
        {
            Step_Max_Z = leaf.getZ();
        }
    }

    //Calculando o Z médio do degrau
    Step_Mean_Z = (Step_Min_Z + Step_Max_Z)/2;

}


void diane_octomap::Step::CalculateStepPropertiesWithMatrix()
{
    //Ordenando as folhas do degrau por Z
    SortLeafMatrixByZInStep();

    //Obtendo o menor Z e o maior Z das folhas contidas no degrau
    Step_Min_Z = Leafs_Of_Step(2, 0);
    Step_Max_Z = Leafs_Of_Step(2, Leafs_Of_Step.cols() - 1);


    //Calculando o Z médio do degrau
    Step_Mean_Z = (Step_Min_Z + Step_Max_Z)/2;

}


void diane_octomap::Step::SortLeafMatrixByZInStep()
{
    //Obtendo a ordem dos índices ordenados pelo Z das folhas
    MatrixXi Indexes;
    MatrixXf SortedMatrix;

    igl::sort(Leafs_Of_Step.row(2), 2, true, SortedMatrix, Indexes);


    igl::slice(Leafs_Of_Step.transpose(), Indexes, 1, SortedMatrix);
    SortedMatrix.transposeInPlace();

    Leafs_Of_Step = SortedMatrix;

}


void diane_octomap::Step::StepHeightFilter()
{
    //Separando as folhas desse degrau em colunas (Mesmo x e y)
    vector<vector<OcTree::leaf_bbx_iterator>> LeafGroups;

    for(int i=0; i<Leafs_In_Step.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = Leafs_In_Step.at(i);

        bool leaf_group_found = false;

        for(int j=0; j<LeafGroups.size(); j++)
        {
            vector<OcTree::leaf_bbx_iterator> leaf_group = LeafGroups.at(j);

            if((leaf.getX() == leaf_group.at(0).getX()) && (leaf.getY() == leaf_group.at(0).getY()))
            {
                //Se encontrou um grupo, insere a folha nesse grupo;
                LeafGroups.at(j).push_back(leaf);

                leaf_group_found = true;
                break;
            }
        }

        //Se não encontrou um grupo, insere um novo grupo;
        if(leaf_group_found == false)
        {
            vector<OcTree::leaf_bbx_iterator> NewLeafGroup;
            NewLeafGroup.push_back(leaf);

            LeafGroups.push_back(NewLeafGroup);
        }

    }

    //Após agrupar, obtém somente as colunas com alturas dentro do padrão de degrau
    vector<vector<OcTree::leaf_bbx_iterator>> FilteredLeafGroups;

    for(int k=0; k<LeafGroups.size(); k++)
    {
        double min_leaf_group_z = 100.0;
        double max_leaf_group_z = 0.0;

        double leaf_group_height = 0.0;

        vector<OcTree::leaf_bbx_iterator> LeafGroup = LeafGroups.at(k);
        for(int l=0; l<LeafGroup.size(); l++)
        {
            OcTree::leaf_bbx_iterator leaf = LeafGroup.at(l);
            if(leaf.getZ() < min_leaf_group_z)
            {
                min_leaf_group_z = leaf.getZ();
            }
            if(leaf.getZ() > max_leaf_group_z)
            {
                max_leaf_group_z = leaf.getZ();
            }
        }

        leaf_group_height = (max_leaf_group_z - min_leaf_group_z);

        if((leaf_group_height >= Min_Step_Height) && (leaf_group_height <= Max_Step_Height))
        {
            FilteredLeafGroups.push_back(LeafGroup);
        }

    }

    //Repopulando as folhas do degrau (somente com as colunas que passaram pelo filtro)
    Leafs_In_Step.clear();

    for(int o=0; o<FilteredLeafGroups.size(); o++)
    {
        for(int p=0; p<FilteredLeafGroups.at(o).size(); p++)
        {
            OcTree::leaf_bbx_iterator leaf = FilteredLeafGroups.at(o).at(p);

            Leafs_In_Step.push_back(leaf);

        }

    }

}


void diane_octomap::Step::StepHeightHistogram()
{
    int Hist_Length = 4/0.05;

    vector<int> stepHistogram(Hist_Length, 0);
    vector<int> index_vote;

    for(int i=0; i<Leafs_In_Step.size(); i++)
    {
        double leafZ = Leafs_In_Step.at(i).getZ();
        int index = leafZ/0.05;
        stepHistogram[index] = stepHistogram[index] + 1;
    }

    for(int i=0; i<stepHistogram.size(); i++)
    {
        if (stepHistogram[i]>=20)
        {
            index_vote.push_back(i);
        }
    }

    //Buscando uma sequência de alturas (buscando uma sequência de 4)
    //Se encontrar uma sequência, seria referente à um degrau.
    bool sequence_found = false;

    if (index_vote.size() > 3)
    {
        for(int i=0; i<index_vote.size() - 3; i++)
        {
            if ((index_vote[i]==index_vote[i+1] - 1) && (index_vote[i]==index_vote[i+2] - 2) && (index_vote[i]==index_vote[i+3] - 3))
            {
                sequence_found = true;

                double minZ = index_vote[i] * 0.05;
                double maxZ = (index_vote[i+3] + 1) * 0.05;

                vector <OcTree::leaf_bbx_iterator> newLeafs;

                //Se encontrou uma sequência, mantém somente as folhas que estejam dentro das alturas definidas por essa sequência
                for(int j=0; j<Leafs_In_Step.size(); j++)
                {
                    if((Leafs_In_Step.at(j).getZ() > minZ) && (Leafs_In_Step.at(j).getZ() < maxZ))
                    {
                        newLeafs.push_back(Leafs_In_Step.at(j));
                    }
                }

                Leafs_In_Step = newLeafs;

                break;;
            }
        }
    }

    if (sequence_found==false)
    {
        //Se não encontrou uma sequência, limpa as folhas do degrau, indicando que o degrau não é válido.
        Leafs_In_Step.clear();
    }

}



diane_octomap::Step::~Step()
{

}




///******Métodos da classe Stair*******


diane_octomap::Stair::Stair()
{
    Min_Z = 1000;
    Max_Z = -1000;


    Num_Steps = 0;
    Step_Length = 0;
    Step_Width = 0;
    Step_Height = 0;

}


void diane_octomap::Stair::SortSteps()
{
    vector<Step*> SortedSteps;

    map<double, int> Mean_Index_Map;

    //Sorting the Stairway's Steps in crescent order of the mean Z-coordinate
    for(int i=0; i<Steps.size(); i++)
    {
        Mean_Index_Map.insert(pair<double, int>(Steps.at(i)->Step_Mean_Z, i));
    }

    for(map<double, int>::iterator itr = Mean_Index_Map.begin(); itr != Mean_Index_Map.end(); itr++)
    {
        //Adding the Step in the sorted vector
        SortedSteps.push_back(Steps.at(itr->second));
    }

    Steps = SortedSteps;

}


void diane_octomap::Stair::ExtractLeafsFromSteps()
{
    for(int i=0; i<Steps.size(); i++)
    {
        Step* step = Steps.at(i);

        //Calculando propriedades do degrau (Será utilizado na ordenacão)
        step->CalculateStepProperties();

        for(int j=0; j<step->Leafs_In_Step.size(); j++)
        {
            OcTree::leaf_bbx_iterator leaf = step->Leafs_In_Step.at(j);

            //Verificando se a folha ja faz parte das folhas da escada. Se não fizer, adiciona essa folha na escada
            if((find(Leafs_In_Stair.begin(), Leafs_In_Stair.end(), leaf) != Leafs_In_Stair.end()) == false)
            {
                Leafs_In_Stair.push_back(leaf);
            }

        }

    }

}


void diane_octomap::Stair::ExtractLeafsFromStepsMatrix()
{
    for(int i=0; i<Steps.size(); i++)
    {
        Step* step = Steps.at(i);

        //Calculating the Step properties (will be used in the sorting)
        step->CalculateStepPropertiesWithMatrix();

        for(int j=0; j<step->Leafs_Of_Step.cols(); j++)
        {
            Vector3f leaf;
            leaf(0,0) = step->Leafs_Of_Step(0, j);
            leaf(1,0) = step->Leafs_Of_Step(1, j);
            leaf(2,0) = step->Leafs_Of_Step(2, j);

            if(Leafs_Of_Stair.cols() == 0)
            {
                int actual_size = Leafs_Of_Stair.cols();

                Leafs_Of_Stair.conservativeResize(3, actual_size + 1);
                Leafs_Of_Stair(0, actual_size) = leaf(0, 0);
                Leafs_Of_Stair(1, actual_size) = leaf(1, 0);
                Leafs_Of_Stair(2, actual_size) = leaf(2, 0);

            }
            else
            {
                if(StairContainsLeaf(leaf) == false)
                {
                    int actual_size = Leafs_Of_Stair.cols();

                    Leafs_Of_Stair.conservativeResize(3, actual_size + 1);
                    Leafs_Of_Stair(0, actual_size) = leaf(0, 0);
                    Leafs_Of_Stair(1, actual_size) = leaf(1, 0);
                    Leafs_Of_Stair(2, actual_size) = leaf(2, 0);

                }

            }

        }

    }

}


bool diane_octomap::Stair::StairContainsLeaf(Vector3f Leaf)
{
    auto find_row_X = (Leafs_Of_Stair.row(0).array() == Leaf[0]);
    auto find_row_Y = (Leafs_Of_Stair.row(1).array() == Leaf[1]);
    auto find_row_Z = (Leafs_Of_Stair.row(2).array() == Leaf[2]);

    auto merge_result = find_row_X* find_row_Y * find_row_Z;

    bool contains = merge_result.any();

    return contains;
}



void diane_octomap::Stair::CalculateStairProperties()
{
    for(int i=0; i<Leafs_In_Stair.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = Leafs_In_Stair.at(i);

        if(Min_Z > leaf.getZ())
        {
            Min_Z = leaf.getZ();
        }
        if(Max_Z < leaf.getZ())
        {
            Max_Z = leaf.getZ();
        }

    }

}


void diane_octomap::Stair::CalculateStairPropertiesWithMatrix()
{
    //Sorting the Leaf Matrix by Z-coordinate
    SortLeafMatrixByZInStair();

    //Getting the minimum and maximum values of Z-coordinate
    Min_Z = Leafs_Of_Stair(2, 0);
    Max_Z = Leafs_Of_Stair(2, Leafs_Of_Stair.cols() - 1);

}


void diane_octomap::Stair::SortLeafMatrixByZInStair()
{
    //Obtaining the order of indexes, ordered by the Z-coordinates of the leafs
    MatrixXi Indexes;
    MatrixXf SortedMatrix;

    igl::sort(Leafs_Of_Stair.row(2), 2, true, SortedMatrix, Indexes);


    igl::slice(Leafs_Of_Stair.transpose(), Indexes, 1, SortedMatrix);
    SortedMatrix.transposeInPlace();

    Leafs_Of_Stair = SortedMatrix;

}


void diane_octomap::Stair::ModelStair(double Octree_Resolution)
{
    //***Obtendo a altura da escada e a altura media dos degraus dessa escada (O candidato com 4 degraus - o primeiro degrau não passou pelos filtros - está gerando uma altura errada)
    Total_Height = ((Max_Z + Octree_Resolution/2) - (Min_Z - Octree_Resolution/2));
    Step_Height = Total_Height/Num_Steps;


    //***Obtendo a largura (aproximada) da escada e a largura média de cada degrau
    //***Obter o centróide do primeiro degrau (estará muito próximo da face mais externa) e calcular a distância até o plano do último degrau (estará próximo ao final)
    Step* First_Step = Steps.at(0);
    Step* Last_Step = Steps.back();

    vector<double> First_Step_Centroid;
    First_Step_Centroid.clear();

    vector<double> Last_Plane_Param = Last_Step->Step_Plane;



    //Calculando o centróide do primeiro degrau
    double X_Sum = 0.0;
    double Y_Sum = 0.0;
    double Z_Sum = 0.0;

    for(int i=0; i<First_Step->Leafs_In_Step.size(); i++)
    {
        X_Sum = X_Sum + First_Step->Leafs_In_Step.at(i).getX();
        Y_Sum = Y_Sum + First_Step->Leafs_In_Step.at(i).getY();
        Z_Sum = Z_Sum + First_Step->Leafs_In_Step.at(i).getZ();
    }

    First_Step_Centroid.push_back((X_Sum)/First_Step->Leafs_In_Step.size());
    First_Step_Centroid.push_back((Y_Sum)/First_Step->Leafs_In_Step.size());
    First_Step_Centroid.push_back((Z_Sum)/First_Step->Leafs_In_Step.size());

    //Obtendo a distância do centróide até o plano do último degrau
    double rho = Last_Plane_Param.at(0);
    double theta = Last_Plane_Param.at(1) * (M_PI/180);
    double phi = Last_Plane_Param.at(2) * (M_PI/180);


    //Definindo a normal do plano do último degrau
    double n[3];
    n[0] = cos(theta)*sin(phi);
    n[1] = sin(theta)*sin(phi);
    n[2] = cos(phi);

    double First_Centroid_Rho = First_Step_Centroid.at(0) * n[0] + First_Step_Centroid.at(1) * n[1] + First_Step_Centroid.at(2) * n[2];

    double Partial_Width = abs(First_Centroid_Rho - rho);
    Step_Width = Partial_Width/(Num_Steps - 1);
    Total_Width = Step_Width*Num_Steps; //Desconsiderando que o último degrau terá uma largura maior




    //Obtendo o par de pontos (1 do 1o degrau e 1 do último degrau) que possuem a maior distância entre si
    //Essa distância será considerada a diagonal total da escada. Projetar os pontos no z=0. Projetar um dos dois pontos para o plano do outro (paralelo ao plano encontrado)
    //A distância resultante será o comprimento da escada

    //Obtendo o par que possui a maior distância entre si
    vector<vector<double>> Selected_Points;

    double Points_Distance = 0.0;

    for(int i=0; i<First_Step->Leafs_In_Step.size(); i++)
    {
        OcTree::leaf_bbx_iterator First_Leaf = First_Step->Leafs_In_Step.at(i);
        double First_X = First_Leaf.getX();
        double First_Y = First_Leaf.getY();
        double First_Z = First_Leaf.getZ();

        vector<double> First_Point;
        First_Point.push_back(First_X);
        First_Point.push_back(First_Y);
        First_Point.push_back(First_Z);

        for(int j=0; j<Last_Step->Leafs_In_Step.size(); j++)
        {
            OcTree::leaf_bbx_iterator Last_Leaf = Last_Step->Leafs_In_Step.at(j);
            double Last_X = Last_Leaf.getX();
            double Last_Y = Last_Leaf.getY();
            double Last_Z = Last_Leaf.getZ();

            vector<double> Last_Point;
            Last_Point.push_back(Last_X);
            Last_Point.push_back(Last_Y);
            Last_Point.push_back(Last_Z);

            double New_Distance = sqrt(pow((Last_X - First_X), 2) + pow((Last_Y - First_Y), 2) + pow((Last_Z - First_Z), 2));

            if(New_Distance > Points_Distance)
            {
                Selected_Points.clear();
                Selected_Points.push_back(First_Point);
                Selected_Points.push_back(Last_Point);

                Points_Distance = New_Distance;
            }

        }

    }

    //Projetando os dois pontos no plano que passa pela centróide do primeiro degrau
    double First_Point_Rho = Selected_Points.at(0).at(0) * n[0] + Selected_Points.at(0).at(1) * n[1] + Selected_Points.at(0).at(2) * n[2];
    double First_Dist = abs(First_Point_Rho - First_Centroid_Rho);

    vector<double> First_Projection_Centroid_Vector;
    First_Projection_Centroid_Vector.push_back(First_Dist * n[0]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[1]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[2]);


    vector<double> First_Projected_Centroid_Point;
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(0) - First_Projection_Centroid_Vector.at(0));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(1) - First_Projection_Centroid_Vector.at(1));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(2) - First_Projection_Centroid_Vector.at(2));


    double Last_Point_Rho = Selected_Points.at(1).at(0) * n[0] + Selected_Points.at(1).at(1) * n[1] + Selected_Points.at(1).at(2) * n[2];
    double Last_Dist = abs(Last_Point_Rho - First_Centroid_Rho);

    vector<double> Last_Projection_Centroid_Vector;
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[0]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[1]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[2]);


    vector<double> Last_Projected_Centroid_Point;
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(0) - Last_Projection_Centroid_Vector.at(0));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(1) - Last_Projection_Centroid_Vector.at(1));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(2) - Last_Projection_Centroid_Vector.at(2));


    //Projetando os dois pontos resultantes no plano horizontal do menor nível detectado de escada (subtraindo metade da resolucao para chegar teoricamente ao chão)
    double n_min[3];
    n_min[0] = 0;
    n_min[1] = 0;
    n_min[2] = 1;

    double First_Projected_Point_Rho = First_Projected_Centroid_Point.at(0) * n_min[0] + First_Projected_Centroid_Point.at(1) * n_min[1] + First_Projected_Centroid_Point.at(2) * n_min[2];
    double First_Min_Dist = abs(First_Projected_Point_Rho - (Min_Z - Octree_Resolution/2));

    vector<double> First_Projection_Vector;
    First_Projection_Vector.push_back(First_Min_Dist * n_min[0]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[1]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[2]);


    vector<double> First_Projected_Point;
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(0) - First_Projection_Vector.at(0));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(1) - First_Projection_Vector.at(1));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(2) - First_Projection_Vector.at(2));


    double Last_Projected_Point_Rho = Last_Projected_Centroid_Point.at(0) * n_min[0] + Last_Projected_Centroid_Point.at(1) * n_min[1] + Last_Projected_Centroid_Point.at(2) * n_min[2];
    double Last_Min_Dist = abs(Last_Projected_Point_Rho - (Min_Z - Octree_Resolution/2));

    vector<double> Last_Projection_Vector;
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[0]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[1]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[2]);


    vector<double> Last_Projected_Point;
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(0) - Last_Projection_Vector.at(0));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(1) - Last_Projection_Vector.at(1));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(2) - Last_Projection_Vector.at(2));


    //Calculando o comprimento da escada e definindo a aresta inicial (inferior do 1o degrau)
    Total_Length = sqrt(pow(abs(First_Projected_Point.at(0) - Last_Projected_Point.at(0)), 2) + pow(abs(First_Projected_Point.at(1) - Last_Projected_Point.at(1)), 2));
    Step_Length = Total_Length;

    Aresta.push_back(First_Projected_Point);
    Aresta.push_back(Last_Projected_Point);



    //Encontrando a direcao para onde a largura deve ser adicionada
    double n_dir[2];
    n_dir[0] = cos(theta);
    n_dir[1] = sin(theta);


    //Calculando os pontos que definem a escada
    vector<double> NewPointA = First_Projected_Point;
    vector<double> NewPointB = Last_Projected_Point;

    Points.push_back(NewPointA);
    Points.push_back(NewPointB);

    for(int i=0; i<Num_Steps; i++)
    {
        //Criando os pontos das quinas (somando a altura --- sempre em Z)
        NewPointA.at(2) = NewPointA.at(2) + Step_Height;
        NewPointB.at(2) = NewPointB.at(2) + Step_Height;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

        //Alterando o X e Y dos pontos para que se desloquem paralelamente ao plano
        NewPointA.at(0) = NewPointA.at(0) + n_dir[0]*Step_Width;
        NewPointA.at(1) = NewPointA.at(1) + n_dir[1]*Step_Width;

        NewPointB.at(0) = NewPointB.at(0) + n_dir[0]*Step_Width;
        NewPointB.at(1) = NewPointB.at(1) + n_dir[1]*Step_Width;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

    }



    //Calculando a angulacão do plano da escada (em graus)
    Plane_Alpha = atan(Step_Height/Step_Width) * (180/M_PI);
    if(Plane_Alpha > 90)
    {
        Plane_Alpha = abs(180 - Plane_Alpha);
    }


}


void diane_octomap::Stair::ModelStair2D(double Octree_Resolution)
{
    //***Obtendo a altura da escada e a altura media dos degraus dessa escada (O candidato com 4 degraus - o primeiro degrau não passou pelos filtros - está gerando uma altura errada)
    Total_Height = ((Max_Z + Octree_Resolution/2) - (Min_Z - Octree_Resolution/2));
    Step_Height = Total_Height/Num_Steps;


    //***Obtendo a largura (aproximada) da escada e a largura média de cada degrau
    //***Obter o centróide do primeiro degrau (estará muito próximo da face mais externa) e calcular a distância até o plano do último degrau (estará próximo ao final)
    Step* First_Step = Steps.at(0);
    Step* Last_Step = Steps.back();

    vector<double> First_Step_Centroid;
    First_Step_Centroid.clear();

    vector<double> Last_Plane_Param;
    Last_Plane_Param.push_back(Last_Step->Step_Line->Line_Rho);
    Last_Plane_Param.push_back(Last_Step->Step_Line->Line_Theta);
    Last_Plane_Param.push_back(90);


    //Calculando o centróide do primeiro degrau
    double X_Sum = 0.0;
    double Y_Sum = 0.0;
    double Z_Sum = 0.0;

    for(int i=0; i<First_Step->Leafs_In_Step.size(); i++)
    {
        X_Sum = X_Sum + First_Step->Leafs_In_Step.at(i).getX();
        Y_Sum = Y_Sum + First_Step->Leafs_In_Step.at(i).getY();
        Z_Sum = Z_Sum + First_Step->Leafs_In_Step.at(i).getZ();
    }

    First_Step_Centroid.push_back((X_Sum)/First_Step->Leafs_In_Step.size());
    First_Step_Centroid.push_back((Y_Sum)/First_Step->Leafs_In_Step.size());
    First_Step_Centroid.push_back((Z_Sum)/First_Step->Leafs_In_Step.size());

    //Obtendo a distância do centróide até o plano do último degrau
    double rho = Last_Plane_Param.at(0);
    double theta = Last_Plane_Param.at(1) * (M_PI/180);
    double phi = Last_Plane_Param.at(2) * (M_PI/180);


    //Definindo a normal do plano do último degrau
    double n[3];
    n[0] = cos(theta)*sin(phi);
    n[1] = sin(theta)*sin(phi);
    n[2] = cos(phi);

    double First_Centroid_Rho = First_Step_Centroid.at(0) * n[0] + First_Step_Centroid.at(1) * n[1] + First_Step_Centroid.at(2) * n[2];

    double Partial_Width = abs(First_Centroid_Rho - rho);
    Step_Width = Partial_Width/(Num_Steps - 1);
    Total_Width = Step_Width*Num_Steps; //Desconsiderando que o último degrau terá uma largura maior




    //Obtendo o par de pontos (1 do 1o degrau e 1 do último degrau) que possuem a maior distância entre si
    //Essa distância será considerada a diagonal total da escada. Projetar os pontos no z=0. Projetar um dos dois pontos para o plano do outro (paralelo ao plano encontrado)
    //A distância resultante será o comprimento da escada

    //Obtendo o par que possui a maior distância entre si
    vector<vector<double>> Selected_Points;

    double Points_Distance = 0.0;

    for(int i=0; i<First_Step->Leafs_In_Step.size(); i++)
    {
        OcTree::leaf_bbx_iterator First_Leaf = First_Step->Leafs_In_Step.at(i);
        double First_X = First_Leaf.getX();
        double First_Y = First_Leaf.getY();
        double First_Z = First_Leaf.getZ();

        vector<double> First_Point;
        First_Point.push_back(First_X);
        First_Point.push_back(First_Y);
        First_Point.push_back(First_Z);

        for(int j=0; j<Last_Step->Leafs_In_Step.size(); j++)
        {
            OcTree::leaf_bbx_iterator Last_Leaf = Last_Step->Leafs_In_Step.at(j);
            double Last_X = Last_Leaf.getX();
            double Last_Y = Last_Leaf.getY();
            double Last_Z = Last_Leaf.getZ();

            vector<double> Last_Point;
            Last_Point.push_back(Last_X);
            Last_Point.push_back(Last_Y);
            Last_Point.push_back(Last_Z);

            double New_Distance = sqrt(pow((Last_X - First_X), 2) + pow((Last_Y - First_Y), 2) + pow((Last_Z - First_Z), 2));

            if(New_Distance > Points_Distance)
            {
                Selected_Points.clear();
                Selected_Points.push_back(First_Point);
                Selected_Points.push_back(Last_Point);

                Points_Distance = New_Distance;
            }

        }

    }

    //Projetando os dois pontos no plano que passa pela centróide do primeiro degrau
    double First_Point_Rho = Selected_Points.at(0).at(0) * n[0] + Selected_Points.at(0).at(1) * n[1] + Selected_Points.at(0).at(2) * n[2];
    double First_Dist = First_Centroid_Rho - First_Point_Rho;
//    double First_Dist = abs(First_Point_Rho - First_Centroid_Rho);

    vector<double> First_Projection_Centroid_Vector;
    First_Projection_Centroid_Vector.push_back(First_Dist * n[0]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[1]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[2]);


    vector<double> First_Projected_Centroid_Point;
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(0) + First_Projection_Centroid_Vector.at(0));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(1) + First_Projection_Centroid_Vector.at(1));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(2) + First_Projection_Centroid_Vector.at(2));


    double Last_Point_Rho = Selected_Points.at(1).at(0) * n[0] + Selected_Points.at(1).at(1) * n[1] + Selected_Points.at(1).at(2) * n[2];
    double Last_Dist = First_Centroid_Rho - Last_Point_Rho;

    vector<double> Last_Projection_Centroid_Vector;
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[0]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[1]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[2]);


    vector<double> Last_Projected_Centroid_Point;
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(0) + Last_Projection_Centroid_Vector.at(0));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(1) + Last_Projection_Centroid_Vector.at(1));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(2) + Last_Projection_Centroid_Vector.at(2));


    //Projetando os dois pontos resultantes no plano horizontal do menor nível detectado de escada (subtraindo metade da resolucao para chegar teoricamente ao chão)
    double n_min[3];
    n_min[0] = 0;
    n_min[1] = 0;
    n_min[2] = 1;

    double First_Projected_Point_Rho = First_Projected_Centroid_Point.at(0) * n_min[0] + First_Projected_Centroid_Point.at(1) * n_min[1] + First_Projected_Centroid_Point.at(2) * n_min[2];
    double First_Min_Dist = (Min_Z - Octree_Resolution/2) - First_Projected_Point_Rho;

    vector<double> First_Projection_Vector;
    First_Projection_Vector.push_back(First_Min_Dist * n_min[0]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[1]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[2]);


    vector<double> First_Projected_Point;
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(0) + First_Projection_Vector.at(0));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(1) + First_Projection_Vector.at(1));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(2) + First_Projection_Vector.at(2));


    double Last_Projected_Point_Rho = Last_Projected_Centroid_Point.at(0) * n_min[0] + Last_Projected_Centroid_Point.at(1) * n_min[1] + Last_Projected_Centroid_Point.at(2) * n_min[2];
    double Last_Min_Dist = (Min_Z - Octree_Resolution/2) - Last_Projected_Point_Rho;

    vector<double> Last_Projection_Vector;
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[0]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[1]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[2]);


    vector<double> Last_Projected_Point;
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(0) + Last_Projection_Vector.at(0));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(1) + Last_Projection_Vector.at(1));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(2) + Last_Projection_Vector.at(2));


    //Calculando o comprimento da escada e definindo a aresta inicial (inferior do 1o degrau)
    Total_Length = sqrt(pow(abs(First_Projected_Point.at(0) - Last_Projected_Point.at(0)), 2) + pow(abs(First_Projected_Point.at(1) - Last_Projected_Point.at(1)), 2));
    Step_Length = Total_Length;

    Aresta.push_back(First_Projected_Point);
    Aresta.push_back(Last_Projected_Point);



    //Encontrando a direcao para onde a largura deve ser adicionada
    double n_dir[2];
    n_dir[0] = cos(theta);
    n_dir[1] = sin(theta);


    //Calculando os pontos que definem a escada
    vector<double> NewPointA = First_Projected_Point;
    vector<double> NewPointB = Last_Projected_Point;

    Points.push_back(NewPointA);
    Points.push_back(NewPointB);

    for(int i=0; i<Num_Steps; i++)
    {
        //Criando os pontos das quinas (somando a altura --- sempre em Z)
        NewPointA.at(2) = NewPointA.at(2) + Step_Height;
        NewPointB.at(2) = NewPointB.at(2) + Step_Height;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

        //Alterando o X e Y dos pontos para que se desloquem paralelamente ao plano
        NewPointA.at(0) = NewPointA.at(0) + n_dir[0]*Step_Width;
        NewPointA.at(1) = NewPointA.at(1) + n_dir[1]*Step_Width;

        NewPointB.at(0) = NewPointB.at(0) + n_dir[0]*Step_Width;
        NewPointB.at(1) = NewPointB.at(1) + n_dir[1]*Step_Width;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

    }



    //Calculando a angulacão do plano da escada (em graus)
    Plane_Alpha = atan(Step_Height/Step_Width) * (180/M_PI);
    if(Plane_Alpha > 90)
    {
        Plane_Alpha = abs(180 - Plane_Alpha);
    }


}


void diane_octomap::Stair::ModelStair2DWithMatrix(double Octree_Resolution)
{
    //Obtaining the stairway total height, the mean height of the stairway's Steps
    Total_Height = ((Max_Z + Octree_Resolution/2) - (Min_Z - Octree_Resolution/2));
    Step_Height = Total_Height/Num_Steps;


    //Obtaining the total width of the stairway and and the mean width of the Steps
    //Obtain the centroid of the first Step (it will be really close to the extern face) and evaluate the distance to the plane of the last step.
    Step* First_Step = Steps.at(0);
    Step* Last_Step = Steps.back();

    vector<double> First_Step_Centroid;
    First_Step_Centroid.clear();

    vector<double> Last_Plane_Param;
    Last_Plane_Param.push_back(Last_Step->Step_Line->Line_Rho);
    Last_Plane_Param.push_back(Last_Step->Step_Line->Line_Theta);
    Last_Plane_Param.push_back(90);


    //Getting the centroid of the first step
    double X_Sum = 0.0;
    double Y_Sum = 0.0;
    double Z_Sum = 0.0;

    X_Sum = First_Step->Leafs_Of_Step.row(0).sum();
    Y_Sum = First_Step->Leafs_Of_Step.row(1).sum();
    Z_Sum = First_Step->Leafs_Of_Step.row(2).sum();

    First_Step_Centroid.push_back((X_Sum)/First_Step->Leafs_Of_Step.cols());
    First_Step_Centroid.push_back((Y_Sum)/First_Step->Leafs_Of_Step.cols());
    First_Step_Centroid.push_back((Z_Sum)/First_Step->Leafs_Of_Step.cols());


    //Getting the distance of the centroid to the plane of the last step
    double rho = Last_Plane_Param.at(0);
    double theta = Last_Plane_Param.at(1) * (M_PI/180);
    double phi = Last_Plane_Param.at(2) * (M_PI/180);


    //Defining the normal vector of the plane of the last step
    double n[3];
    n[0] = cos(theta)*sin(phi);
    n[1] = sin(theta)*sin(phi);
    n[2] = cos(phi);

    double First_Centroid_Rho = First_Step_Centroid.at(0) * n[0] + First_Step_Centroid.at(1) * n[1] + First_Step_Centroid.at(2) * n[2];

    double Partial_Width = abs(First_Centroid_Rho - rho);
    Step_Width = Partial_Width/(Num_Steps - 1);
    Total_Width = Step_Width*Num_Steps; //Not considering that the last step will have a larger width



    //Obtaining the pair of points(1 of the first step and 1 of the last step) that are the most farther from one another
    //This distance will be considered as the total diagonal of the stair. Project the points in z=0. Project one of the points to the plane of the other
    //The resulting distance will be the length of the stair

    //Obtaining the pair that are the most farther from one another
    vector<vector<double>> Selected_Points;

    double Points_Distance = 0.0;

    for(int i=0; i<First_Step->Leafs_Of_Step.cols(); i++)
    {
        double First_X = First_Step->Leafs_Of_Step(0, i);
        double First_Y = First_Step->Leafs_Of_Step(1, i);
        double First_Z = First_Step->Leafs_Of_Step(2, i);

        vector<double> First_Point;
        First_Point.push_back(First_X);
        First_Point.push_back(First_Y);
        First_Point.push_back(First_Z);

        for(int j=0; j<Last_Step->Leafs_Of_Step.cols(); j++)
        {
            double Last_X = Last_Step->Leafs_Of_Step(0, j);
            double Last_Y = Last_Step->Leafs_Of_Step(1, j);
            double Last_Z = Last_Step->Leafs_Of_Step(2, j);

            vector<double> Last_Point;
            Last_Point.push_back(Last_X);
            Last_Point.push_back(Last_Y);
            Last_Point.push_back(Last_Z);

            double New_Distance = sqrt(pow((Last_X - First_X), 2) + pow((Last_Y - First_Y), 2) + pow((Last_Z - First_Z), 2));

            if(New_Distance > Points_Distance)
            {
                Selected_Points.clear();
                Selected_Points.push_back(First_Point);
                Selected_Points.push_back(Last_Point);

                Points_Distance = New_Distance;
            }

        }

    }


    //Projecting the two points in the vertical plane that passes through the centroid of the first step
    double First_Point_Rho = Selected_Points.at(0).at(0) * n[0] + Selected_Points.at(0).at(1) * n[1] + Selected_Points.at(0).at(2) * n[2];
//    double First_Dist = abs(First_Point_Rho - First_Centroid_Rho);
    double First_Dist = First_Centroid_Rho - First_Point_Rho;

    vector<double> First_Projection_Centroid_Vector;
    First_Projection_Centroid_Vector.push_back(First_Dist * n[0]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[1]);
    First_Projection_Centroid_Vector.push_back(First_Dist * n[2]);


    vector<double> First_Projected_Centroid_Point;
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(0) + First_Projection_Centroid_Vector.at(0));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(1) + First_Projection_Centroid_Vector.at(1));
    First_Projected_Centroid_Point.push_back(Selected_Points.at(0).at(2) + First_Projection_Centroid_Vector.at(2));


    double Last_Point_Rho = Selected_Points.at(1).at(0) * n[0] + Selected_Points.at(1).at(1) * n[1] + Selected_Points.at(1).at(2) * n[2];
//    double Last_Dist = abs(Last_Point_Rho - First_Centroid_Rho);
    double Last_Dist = First_Centroid_Rho - Last_Point_Rho;

    vector<double> Last_Projection_Centroid_Vector;
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[0]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[1]);
    Last_Projection_Centroid_Vector.push_back(Last_Dist * n[2]);


    vector<double> Last_Projected_Centroid_Point;
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(0) + Last_Projection_Centroid_Vector.at(0));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(1) + Last_Projection_Centroid_Vector.at(1));
    Last_Projected_Centroid_Point.push_back(Selected_Points.at(1).at(2) + Last_Projection_Centroid_Vector.at(2));


    //Projecting the two resultant points in the horizontal plane of the lower detected level of the stair (subtracting half the resolution to obtain points in the floor)
    double n_min[3];
    n_min[0] = 0;
    n_min[1] = 0;
    n_min[2] = 1;

    double First_Projected_Point_Rho = First_Projected_Centroid_Point.at(0) * n_min[0] + First_Projected_Centroid_Point.at(1) * n_min[1] + First_Projected_Centroid_Point.at(2) * n_min[2];
//    double First_Min_Dist = abs(First_Projected_Point_Rho - (Min_Z - Octree_Resolution/2));
    double First_Min_Dist = (Min_Z - Octree_Resolution/2) - First_Projected_Point_Rho;

    vector<double> First_Projection_Vector;
    First_Projection_Vector.push_back(First_Min_Dist * n_min[0]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[1]);
    First_Projection_Vector.push_back(First_Min_Dist * n_min[2]);


    vector<double> First_Projected_Point;
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(0) + First_Projection_Vector.at(0));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(1) + First_Projection_Vector.at(1));
    First_Projected_Point.push_back(First_Projected_Centroid_Point.at(2) + First_Projection_Vector.at(2));


    double Last_Projected_Point_Rho = Last_Projected_Centroid_Point.at(0) * n_min[0] + Last_Projected_Centroid_Point.at(1) * n_min[1] + Last_Projected_Centroid_Point.at(2) * n_min[2];
//    double Last_Min_Dist = abs(Last_Projected_Point_Rho - (Min_Z - Octree_Resolution/2));
    double Last_Min_Dist = (Min_Z - Octree_Resolution/2) - Last_Projected_Point_Rho;

    vector<double> Last_Projection_Vector;
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[0]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[1]);
    Last_Projection_Vector.push_back(Last_Min_Dist * n_min[2]);


    vector<double> Last_Projected_Point;
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(0) + Last_Projection_Vector.at(0));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(1) + Last_Projection_Vector.at(1));
    Last_Projected_Point.push_back(Last_Projected_Centroid_Point.at(2) + Last_Projection_Vector.at(2));


    //Calculating the length of the stair and defining the inicial line (inferior of the first step)
    Total_Length = sqrt(pow(abs(First_Projected_Point.at(0) - Last_Projected_Point.at(0)), 2) + pow(abs(First_Projected_Point.at(1) - Last_Projected_Point.at(1)), 2));
    Step_Length = Total_Length;

    Aresta.push_back(First_Projected_Point);
    Aresta.push_back(Last_Projected_Point);



    //Finding the direction to where the width must be added
    double n_dir[2];
    n_dir[0] = cos(theta);
    n_dir[1] = sin(theta);


    //Calculainting the points that define the stair
    vector<double> NewPointA = First_Projected_Point;
    vector<double> NewPointB = Last_Projected_Point;

    Points.push_back(NewPointA);
    Points.push_back(NewPointB);

    for(int i=0; i<Num_Steps; i++)
    {
        //Creating the points of the edges
        NewPointA.at(2) = NewPointA.at(2) + Step_Height;
        NewPointB.at(2) = NewPointB.at(2) + Step_Height;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

        //Altering (X, Y) coordinates to move alongside the direction found
        NewPointA.at(0) = NewPointA.at(0) + n_dir[0]*Step_Width;
        NewPointA.at(1) = NewPointA.at(1) + n_dir[1]*Step_Width;

        NewPointB.at(0) = NewPointB.at(0) + n_dir[0]*Step_Width;
        NewPointB.at(1) = NewPointB.at(1) + n_dir[1]*Step_Width;

        Points.push_back(NewPointA);
        Points.push_back(NewPointB);

    }



    //Calculating the stair alpha value (in degrees)
    Plane_Alpha = atan(Step_Height/Step_Width) * (180/M_PI);
    if(Plane_Alpha > 90)
    {
        Plane_Alpha = abs(180 - Plane_Alpha);
    }


}


diane_octomap::Stair::~Stair()
{

}


///******Métodos da classe Line*******


diane_octomap::Line::Line()
{
    Leafs_Of_Line = MatrixXf();

    Line_Rho = 0;
    Line_Theta = 0;
    Line_Votes = 0;
    Line_Z = 0;

    min_X = 1000;
    max_X = -1000;
    min_Z = 1000;
    max_Z = -1000;
}


//Atualizando os limites de X e Z do objeto Line
void diane_octomap::Line::UpdateLimitsWithMatrix()
{
    if(Leafs_Of_Line.cols() > 0)
    {
        //Ordenando a matriz de folhas por X (para acessar o X mínimo e o X máximo)
        SortLeafMatrixByX();

        min_X = Leafs_Of_Line(0, 0);
        max_X = Leafs_Of_Line(0, Leafs_Of_Line.cols() - 1);

        //Ordenando a matriz de folhas por Z (para acessar o Z mínimo e o Z máximo)
        SortLeafMatrixByZ();

        min_Z = Leafs_Of_Line(2, 0);
        max_Z = Leafs_Of_Line(2, Leafs_Of_Line.cols() - 1);

    }

}


//Ordenando a matriz de folhas por X
void diane_octomap::Line::SortLeafMatrixByX()
{
    //Obtendo a ordem dos índices ordenados pelo X das folhas
    MatrixXi Indexes;
    MatrixXf SortedMatrix;

    igl::sort(Leafs_Of_Line.row(0), 2, true, SortedMatrix, Indexes);


    igl::slice(Leafs_Of_Line.transpose(), Indexes, 1, SortedMatrix);
    SortedMatrix.transposeInPlace();

    Leafs_Of_Line = SortedMatrix;

}


//Ordenando a matriz de folhas por Z
void diane_octomap::Line::SortLeafMatrixByZ()
{
    //Obtendo a ordem dos índices ordenados pelo Z das folhas
    MatrixXi Indexes;
    MatrixXf SortedMatrix;

    igl::sort(Leafs_Of_Line.row(2), 2, true, SortedMatrix, Indexes);


    igl::slice(Leafs_Of_Line.transpose(), Indexes, 1, SortedMatrix);
    SortedMatrix.transposeInPlace();

    Leafs_Of_Line = SortedMatrix;

}


void diane_octomap::Line::UpdateLineParametersWithMinSquare()
{
    double NewLineRho = 0.0;
    double NewLineTheta = 0.0;

    MatrixXf A = MatrixXf(Leafs_Of_Line.cols(), 2);
    MatrixXf B = MatrixXf(Leafs_Of_Line.cols(), 1);

    //Gerando as matrizes A e B
    for(int i=0; i<Leafs_Of_Line.cols(); ++i)
    {
        //Gerando a Matriz A ([1 x0; 1 x1; ... ; 1 xn])
        A(i, 0) = 1;
        A(i, 1) = Leafs_Of_Line(0, i);

        //Gerando a Matriz B ([y0; y1; ... ; yn])
        B(i, 0) = Leafs_Of_Line(1, i);
    }

    //Utilizando mínimos quadrados para obter os coeficientes [b, a]
    VectorXf Line_Coef =  A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);

    NewLineTheta = atan((-1)*(Line_Coef(0,0)/Line_Coef(1,0))/(Line_Coef(0,0))) * 180/M_PI;
    NewLineRho = Line_Coef(0,0)*sin(NewLineTheta * M_PI/180);

    //Caso o Rho calculado dê negativo, altera o Rho e Theta
    if(NewLineRho < 0)
    {
        NewLineRho = abs(NewLineRho);
        NewLineTheta = NewLineTheta + 180;
    }

    Line_Rho = NewLineRho;
    Line_Theta = NewLineTheta;

}


diane_octomap::Line::~Line()
{

}
