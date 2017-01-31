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

    //Variáveis para filtrar os planos a serem extraídos
    Filter_Phi_Min = 85;
    Filter_Phi_Max = 96;
    Filter_Vote_Min = 250;
    Filter_Vote_Max = 400;

    //Definindo as tolerâncias para o merge dos planos
    delta_merge_Rho = 0.11;

    delta_Rho = 0.1;
    delta_Theta = 0;
    delta_Phi = 0;

    //Definindo características da escada
    Min_Num_Steps = 3;
    Min_Step_Width = 0.25;
    Max_Step_Width = 0.35;
    Min_Step_Height = 0.09;
    Max_Step_Height = 0.20;
}


void diane_octomap::DianeOctomap::onInit()
{
    StartInternalCycle();
}


void diane_octomap::DianeOctomap::StartInternalCycle()
{
    mutStartStop.lock();

    stop = false;

    //Obtendo a octree à partir do arquivo (o caminho para o arquivo ainda está definido chapado no código - mudar para um arquivo de configuracão).
    DianeOctomap::GenerateOcTreeFromFile();

    //Filtrando e armazenando as folhas da octree que estejam dentro da Bounding Box definida no método e que estejam ocupadas.
    DianeOctomap::GetOccupiedLeafsOfBBX(octree);

    //Utilizando as folhas filtradas (presentes no vetor) para detectar as informacões da escada.
//    DianeOctomap::StairDetection();
    DianeOctomap::StairDetection2d();

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


void diane_octomap::DianeOctomap::GenerateOcTreeFromFile()
{
    string otFileName = "/home/rob/catkin_ws/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_Inclinada_5.ot";
//    string otFileName = "/home/rob/catkin_ws/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_5.ot";

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


void diane_octomap::DianeOctomap::GetOccupiedLeafsOfBBX(OcTree* octree)
{
//    point3d min;
//    min.x() = -0.70;
//    min.y() = 0;
//    min.z() = 0;

//    point3d max;
//    max.x() = 0.65;
//    max.y() = 100;
//    max.z() = 100;

    point3d min;
    min.x() = -5;
    min.y() = -5;
    min.z() = -5;

    point3d max;
    max.x() = 5;
    max.y() = 100;
    max.z() = 100;

    //Armazenando somente as folhas pertencentes à Bounding Box e que estejam ocupadas.
    for(OcTree::leaf_bbx_iterator bbx_it = octree->begin_leafs_bbx(min, max), end=octree->end_leafs_bbx(); bbx_it!= end; ++bbx_it)
    {
        if(octree->isNodeOccupied(*bbx_it))
        {
            OccupiedLeafsInBBX.push_back(bbx_it);
        }
    }

    cout << "Folhas ocupadas: " << OccupiedLeafsInBBX.size() << endl << endl;

}





//Fazendo pelas retas -------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------

void diane_octomap::DianeOctomap::StairDetection2d()
{

    vector<double> parameter= diane_octomap::DianeOctomap::getParameter(OccupiedLeafsInBBX);

    double length = parameter.at(0);
    double width = parameter.at(1);

    vector<vector<OcTree::leaf_bbx_iterator>> Grouped_Leafs = GroupPlanesByZ(OccupiedLeafsInBBX);

    //Hough - encontrando as retas em cada altura em Z
    vector<vector<diane_octomap::Line*>> Lines = LineHoughTransform(length, width, Grouped_Leafs);


    vector<vector<diane_octomap::Line*>> GroupLinesByRhoTheta = GroupLineByRhoTheta(Lines);


    vector<vector<diane_octomap::Line*>> Filtered_Groups = FilterGroups(GroupLinesByRhoTheta, 3, 10);


    vector<diane_octomap::Line*> Merged_Lines = MergeGroupedLines(Filtered_Groups);


    PopulateLines(Merged_Lines, Grouped_Leafs);

    //Segmentando as linhas
    vector<diane_octomap::Line*> Segmented_Lines = segLine(Merged_Lines);


    //Agrupando as linhas por Theta e pelo intervalo do segmento
    vector<vector<Line*>> GroupThetaIntervalLines = GroupLinesByThetaAndInterval(Segmented_Lines);


    //Filtrando os grupos de linhas que não possuem elementos suficientes para formar degraus
    vector<vector<Line*>> Filtered_Segmented_Groups;
    for(int i=0; i<GroupThetaIntervalLines.size(); i++)
    {
        if(GroupThetaIntervalLines.at(i).size() >= Min_Num_Steps)
        {
            Filtered_Segmented_Groups.push_back(GroupThetaIntervalLines.at(i));
        }
    }

    //Aplicando o merge em Rho para cada Grupo
    vector<vector<Line*>> Merged_Segmented_Groups = MergeSegmentedGroupsLines(Filtered_Segmented_Groups);


    //Busca uma sequência válida que seria referente à uma escada

    vector<vector<Line*>> Filtered_Sequence = SequenceFilter(Merged_Segmented_Groups);

    cout<<endl;

}


//Junta todos os planos com o mesmo Z. Com o objetivo de separar por linhas.
vector<vector<OcTree::leaf_bbx_iterator>> diane_octomap::DianeOctomap::GroupPlanesByZ(vector<OcTree::leaf_bbx_iterator> Leafs)
{
    vector<vector<OcTree::leaf_bbx_iterator>>Grouped2d;
    double Z;
    //Separando os planos por Z

    for(int i=0; i<Leafs.size(); i++)
    {
        Z=Leafs.at(i).getZ();

        bool Group_Found = false;

        //Verificando se o Z já faz parte de um grupo, se encontrar o coloca no grupo.
        for(int j=0; j<Grouped2d.size(); j++)
        {
            if((Z == ((Grouped2d.at(j).at(0).getZ()))))
            {
                Grouped2d.at(j).push_back(Leafs.at(i));
                Group_Found = true;
                break;
            }

        }

        if(Group_Found == false)
        {
            //Se o grupo não foi encontrado, cria um novo grupo
            vector<OcTree::leaf_bbx_iterator> NewGroup2d;
            NewGroup2d.push_back(Leafs.at(i));

            Grouped2d.push_back(NewGroup2d);
        }

    }

    return Grouped2d;

}


//Retorna a largura e o comprimento do mapa para contruçao do espaço de Hough
vector<double> diane_octomap::DianeOctomap::getParameter(vector<OcTree::leaf_bbx_iterator> Leafs)
    {
     double max_x, min_x,max_y,min_y;
        for(int i = 0; i < Leafs.size(); i++)
        {

            if(Leafs.at(i).getX() < min_x)
            {
                min_x = Leafs.at(i).getX();
            }
            if(Leafs.at(i).getX() > max_x)
            {
                 max_x = Leafs.at(i).getX();
            }

            if(Leafs.at(i).getY() < min_y)
            {
                min_y = Leafs.at(i).getY();
            }
            if(Leafs.at(i).getY() > max_y)
            {
                max_y = Leafs.at(i).getY();
            }

        }

    double length = max_x - min_x;
    double width = max_y - min_y;
    vector<double> result;
    result.push_back(length);
    result.push_back(width);

    return result;

}


//Cria as retas usando a transformada de Hough
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::LineHoughTransform(double length, double width, vector<vector<OcTree::leaf_bbx_iterator>> Leafs)
{

    vector<vector<Line*>> Group_Lines;

    //Parametros que serao usados para criar o espaço de Hough
    Rho_Min = 0;
    Rho_Max = ceil(sqrt(pow(length, 2) + pow(width, 2)));
    Theta_Min = 0;
    Theta_Max = 360;

    Rho_Passo = 0.05;
    Theta_Passo = 5;

    Rho_Num = (Rho_Max - Rho_Min)/Rho_Passo;
    Theta_Num = (Theta_Max - Theta_Min)/Theta_Passo;

    for (int i=0;i<Leafs.size();i++)
    {

        vector<vector<int>> Votes = AccumulatePoint2d(Leafs.at(i));

        vector<Line*> Lines = createGroupLines(Votes, Leafs.at(i).at(0).getZ());

        Group_Lines.push_back(Lines);
    }

    return Group_Lines;

}


//Cria o espaço de Hough e faz a votaçao para cada Z
vector<vector<int>> diane_octomap::DianeOctomap::AccumulatePoint2d(vector<OcTree::leaf_bbx_iterator> LeafZ)
{
    vector<vector<int>> AccumulatePoint2d;
    vector<int> addAccumulate;
    bool add=false;

    //Cria o espaço de Hough
    for (int j=0;j<Rho_Num;j++)
    {

        if(add==false)
        {
            for(int k=0;k<Theta_Num;k++)
            {
                addAccumulate.push_back(0);
            }
            add=true;
        }

    AccumulatePoint2d.push_back(addAccumulate);


    }


    //Faz a votaçao
    for (int k=0;k<Theta_Num;k++)
    {
        double Theta =(Theta_Min+k*Theta_Passo+Theta_Passo/2) * (M_PI/180);
        double ctheta=cos(Theta);
        double stheta=sin(Theta);
        for(int j=0;j<Rho_Num;j++)
        {
            double Rho=(Rho_Min+(j+0.5)*Rho_Passo);
            for (int l=0;l<LeafZ.size();l++)
                {
                    double Rho_point = LeafZ.at(l).getX()*ctheta+LeafZ.at(l).getY()*stheta;

                    if (fabs(Rho_point-Rho)<=Rho_Passo/2)
                    {
                        AccumulatePoint2d.at(j).at(k)+=1;
                    }
                }
        }



    }

    return AccumulatePoint2d;

}


//Crias as retas olhando para a votaçao que foi feita, e retorna o grupo de todas elas.
vector<diane_octomap::Line*> diane_octomap::DianeOctomap::createGroupLines(vector<vector<int>> Votes, double Z)
{
    vector<Line*> Lines;

    for (int i=0;i<Votes.size();i++)
    {
        for(int j=0;j<Votes.at(i).size();j++)
            //Cria a reta caso tenha algum voto
            if(Votes.at(i).at(j)>10)
            {

                Line* line= new Line();
                line->Line_Rho=(Rho_Min+(i+0.5)*Rho_Passo);
                line->Line_Theta=(Theta_Min+(j+0.5)*Theta_Passo);
                line->Line_Votes=Votes.at(i).at(j);
                line->Line_Z=Z;
                Lines.push_back(line);
            }
    }

    return Lines;

}


vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::GroupLineByRhoTheta(vector<vector<diane_octomap::Line*>> Lines)
{
    vector<vector<diane_octomap::Line*>>GroupedLineByTheta;
    double theta;
    double rho;
    //Separando os planos por Theta

    for(int i=0; i<Lines.size(); i++)
    {
        for (int k=0; k<Lines.at(i).size();k++)
        {
             theta=Lines.at(i).at(k)->Line_Theta;
             rho=Lines.at(i).at(k)->Line_Rho;

             bool Group_Found = false;

             //Verificando se o theta já faz parte de um grupo
             for(int j=0; j<GroupedLineByTheta.size(); j++)
             {
                 if((theta == ((GroupedLineByTheta.at(j).at(0)->Line_Theta))) && (rho==GroupedLineByTheta.at(j).at(0)->Line_Rho))
                 {
                     GroupedLineByTheta.at(j).push_back(Lines.at(i).at(k));

                     Group_Found = true;
                     break;
                 }

             }

             if(Group_Found == false)
             {
                 //Se o grupo não foi encontrado, cria um novo grupo
                 vector<diane_octomap::Line*> NewGroup;
                 NewGroup.push_back(Lines.at(i).at(k));

                 GroupedLineByTheta.push_back(NewGroup);
             }

         }
    }

    return GroupedLineByTheta;

}


//Filtra os grupos no qual o numero de retas possam forma um degrau, no qual o max e min depende da resoluçao do octomap
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::FilterGroups(vector<vector<diane_octomap::Line*>> GroupLineByTheta, int min, int max)
{
    vector<vector<diane_octomap::Line*>> FilterGroups;

    for (int i=0;i<GroupLineByTheta.size();i++)
    {
        if(GroupLineByTheta.at(i).size()>=min && GroupLineByTheta.at(i).size()<=max)
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


void diane_octomap::DianeOctomap::PopulateLines(vector<diane_octomap::Line*>& Merged_Lines, vector<vector<OcTree::leaf_bbx_iterator>> Leaf_Groups)
{
    for(int i=0; i<Merged_Lines.size(); i++)
    {
        double Rho = Merged_Lines.at(i)->Line_Rho;
        double Theta = Merged_Lines.at(i)->Line_Theta * (M_PI/180);
        double ctheta=cos(Theta);
        double stheta=sin(Theta);

        for(int j=0; j< Leaf_Groups.size(); j++)
        {
            vector<OcTree::leaf_bbx_iterator> Leaf_Group = Leaf_Groups.at(j);
            if((Leaf_Group.at(0).getZ() >= Merged_Lines.at(i)->min_Z) && (Leaf_Group.at(0).getZ() <= Merged_Lines.at(i)->max_Z))
            {
                for(int k=0; k<Leaf_Group.size(); k++)
                {
                    OcTree::leaf_bbx_iterator Leaf = Leaf_Group.at(k);

                    double Rho_point = Leaf.getX()*ctheta + Leaf.getY()*stheta;

                    if(fabs(Rho_point-Rho)<=Rho_Passo/2)
                    {
                        if((find(Merged_Lines.at(i)->Leafs_In_Line.begin(), Merged_Lines.at(i)->Leafs_In_Line.end(), Leaf) != Merged_Lines.at(i)->Leafs_In_Line.end()) == false)
                        {
                            Merged_Lines.at(i)->Leafs_In_Line.push_back(Leaf);
                        }

                    }

                }

            }

        }

        //Após popular as folhas, atualiza os limites de X e de Z da linha
        Merged_Lines.at(i)->UpdateLimits();
        Merged_Lines.at(i)->sortLeafs();
    }

}

vector<diane_octomap::Line*> diane_octomap::DianeOctomap::segLine(vector<Line*> Lines)
{
    vector<diane_octomap::Line*> newLine;
    bool create_newline = false;

    int i;
    for (i = 0;i<Lines.size();i++)
    {
        Line* line = Lines.at(i);
        bool break_occ = false;
        for(int j = 0 ; j < line->Leafs_In_Line.size()-1; j++)
        {

            if((fabs(line->Leafs_In_Line.at(j).getX() - line->Leafs_In_Line.at(j+1).getX()))>0.11)
            {
                double x = line->Leafs_In_Line.at(j).getX();
                double y = line->Leafs_In_Line.at(j+1).getX();
                create_newline = true;
                break_occ = true;
                Line* newline1 = new Line();
                Line* newline2 = new Line();

                newline1->Line_Rho = line->Line_Rho;
                newline1->Line_Theta = line->Line_Theta;

                newline2->Line_Rho = line->Line_Rho;
                newline2->Line_Theta = line->Line_Theta;


                for(int k=0;k<=j;k++)
                {
                    OcTree::leaf_bbx_iterator leaf = line->Leafs_In_Line.at(k);
                    newline1->Leafs_In_Line.push_back(leaf);
                }

                for(int r=j+1;r<line->Leafs_In_Line.size();r++)
                {
                    newline2->Leafs_In_Line.push_back(line->Leafs_In_Line.at(r));
                }

                newline1->Line_Votes = newline1->Leafs_In_Line.size();
                newline2->Line_Votes = newline2->Leafs_In_Line.size();

                newline1->UpdateLimits();
                newline2->UpdateLimits();
                newLine.push_back(newline1);
                newLine.push_back(newline2);

                break;
            }

        }

        if(break_occ) {break;}
        newLine.push_back(Lines.at(i));


    }


    for(int k=i+1;k<Lines.size();k++)
    {
        newLine.push_back(Lines.at(k));
    }
    if(create_newline){newLine=segLine(newLine);}
    return newLine;


}


//Agrupando as linhas que possuem o mesmo Theta e pelos limites de X
vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::GroupLinesByThetaAndInterval(vector<diane_octomap::Line*> Segmented_Lines)
{
    vector<vector<Line*>> Groups_Lines;

    for(int i=0; i<Segmented_Lines.size(); i++)
    {
        Line* line = Segmented_Lines.at(i);
        double line_rho = line->Line_Rho;
        double line_theta = line->Line_Theta;   //Em graus

        bool group_found = false;

        for(int j=0; j<Groups_Lines.size(); j++)
        {
            //Se a linha possuir um theta igual ao de um grupo, verifica se o segmento da nova linha possui limites de X condizentes com o da primeira linha do grupo
            if(line_theta == Groups_Lines.at(j).at(0)->Line_Theta)
            {
                //Calculando a normal da reta (para verificar a posicão do intervalo em X na reta da 1a linha)
                double line_normal[2];
                line_normal[0] = cos(line_theta * (M_PI/180));
                line_normal[1] = sin(line_theta * (M_PI/180));

                double dist = fabs(line_rho - Groups_Lines.at(j).at(0)->Line_Rho);

                double temp_min_X = line->min_X - (dist*line_normal[0]);
                double temp_max_X = line->max_X - (dist*line_normal[0]);



                //Se as distâncias entre os mínimos dos intervalos e entre os máximos dos intervalos estiverem dentro da tolerância, adiciona a nova linha nesse grupo
                double X_Tolerance = 0.10;

                if((fabs(temp_min_X - Groups_Lines.at(j).at(0)->min_X) <= X_Tolerance) && (fabs(temp_max_X - Groups_Lines.at(j).at(0)->max_X) <= X_Tolerance))
                {
                    Groups_Lines.at(j).push_back(line);
                    group_found = true;

                    break;
                }

            }

        }

        //Se não encontrou um grupo válido (mesmo Theta e com intervalo dentro dos limites determinados pelo grupo), cria um novo.
        if(group_found == false)
        {
            vector<Line*> Group_Lines;
            Group_Lines.push_back(line);

            //Adicionando o novo grupo
            Groups_Lines.push_back(Group_Lines);
        }

    }

    return Groups_Lines;

}


vector<vector<diane_octomap::Line*>> diane_octomap::DianeOctomap::MergeSegmentedGroupsLines(vector<vector<diane_octomap::Line*>> GroupThetaIntervalLines)
{
    vector<vector<Line*>> Merged_Segmented_Lines;

    //Para cada grupo, aplica o merge de Rho
    for(int i=0; i<GroupThetaIntervalLines.size(); i++)
    {
        vector<Line*> Merged_Lines = MergeSegmentedGroup(GroupThetaIntervalLines.at(i));

        if(Merged_Lines.size() >= Min_Num_Steps)
        {
            //Ordenando as linhas no grupo por Rho
            vector<Line*> Sorted_Lines = SortGroupLines(Merged_Lines);

            //Adicionando o grupo que passou pelo Merge e pelo Sort no retorno
            Merged_Segmented_Lines.push_back(Sorted_Lines);
        }
    }


    return Merged_Segmented_Lines;
}


vector<diane_octomap::Line*> diane_octomap::DianeOctomap::MergeSegmentedGroup(vector<diane_octomap::Line*> SegmentedGroupLines)
{
    vector<Line*> Merged_Lines;

    Line* NewLine = new Line();

    int i = 0, j = 0;


    bool merge_break = false;

    //Buscando dois Lines no grupo que possam fazer merge
    if(SegmentedGroupLines.size() > 0)
    {
        //Para cada Line, verifica se cada um dos planos seguintes podem ser agrupados com o inicial
        for(i=0; i<SegmentedGroupLines.size() - 1; i++)
        {
            for(j=i+1; j<SegmentedGroupLines.size(); j++)
            {
                if(CanMergeLines(SegmentedGroupLines.at(i), SegmentedGroupLines.at(j)))
                {
                    //Se dois Lines forem marcados como mergeable, executa o MergeSegmentedGroup de novo, repassando o novo SegmentedGroupLines (com o novo Line e sem os dois antigos)
                    NewLine = FitLine(SegmentedGroupLines.at(i), SegmentedGroupLines.at(j));
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

        //Copiando os planos antigos para o vetor temporário
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

        //Recursão para obter somente os Lines aglutinados finais
        Merged_Lines = MergeSegmentedGroup(temp_Lines);

    }
    else
    {
        //Se não existirem mais Lines a serem aglutinados, retorna o vetor que foi recebido
        Merged_Lines = SegmentedGroupLines;
    }

    return Merged_Lines;

}


bool diane_octomap::DianeOctomap::CanMergeLines(Line* LineA, Line* LineB)
{

    bool Result = false;

    double RhoA = LineA->Line_Rho;
    double RhoB = LineB->Line_Rho;


    //Se a distância em Rho for menor do que um delta,
    if((fabs(RhoA - RhoB) <= delta_merge_Rho))
    {
        Result = true;
    }


    return Result;

}


diane_octomap::Line* diane_octomap::DianeOctomap::FitLine(diane_octomap::Line* LineA, diane_octomap::Line* LineB)
{
    //Inicialmente, o Line resultante possuirá parâmetros RHO e THETA iguais às médias ponderadas dos parâmetros dos planos recebidos, onde o peso é a quantidade de votos.

    Line* Result_Line = new Line();

    double Rho = (LineA->Line_Rho * LineA->Line_Votes + LineB->Line_Rho * LineB->Line_Votes)/(LineA->Line_Votes + LineB->Line_Votes);
    double Theta = (LineA->Line_Theta * LineA->Line_Votes + LineB->Line_Theta * LineB->Line_Votes)/(LineA->Line_Votes + LineB->Line_Votes);

    if(Theta > 360)
    {
        Theta = remainder(Theta,360);
    }

    double Votes = (LineA->Line_Votes + LineB->Line_Votes);

    Result_Line->Line_Rho = Rho;
    Result_Line->Line_Theta = Theta;
    Result_Line->Line_Votes = Votes;


    //Repassando as folhas dos Lines antigos para o novo Line
    for(int i=0; i<LineA->Leafs_In_Line.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = LineA->Leafs_In_Line.at(i);

        if((find(Result_Line->Leafs_In_Line.begin(), Result_Line->Leafs_In_Line.end(), leaf) != Result_Line->Leafs_In_Line.end()) == false)
        {
            Result_Line->Leafs_In_Line.push_back(leaf);
        }

    }
    for(int j=0; j<LineB->Leafs_In_Line.size(); j++)
    {
        OcTree::leaf_bbx_iterator leaf = LineB->Leafs_In_Line.at(j);

        if((find(Result_Line->Leafs_In_Line.begin(), Result_Line->Leafs_In_Line.end(), leaf) != Result_Line->Leafs_In_Line.end()) == false)
        {
            Result_Line->Leafs_In_Line.push_back(leaf);
        }

    }

    Result_Line->sortLeafs();
    Result_Line->UpdateLimits();


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

    //Se ocorreu um swap, chama novamente o Sort passando o novo grupo
    if(swap_break)
    {
        //Pegando os Lines que estavam antes do 1o (não sofreram swap)

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

        //Recursão para ordenar o novo grupo
        SortedGroup = SortGroupLines(TempGroup);

    }
    else
    {
        SortedGroup = GroupLines;
    }

    return SortedGroup;

}

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
    double dist = 0.35;
    double Tol_Z = 0.1;

    double minZFirstStep = Group_Lines.at(0)->min_Z;

    if(fabs(minZFirstStep) < Tol_Z)
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


                    if(((dist1 >= Min_Step_Width ) && (dist1 <= Max_Step_Width)) && ((dist2 >= Min_Step_Width) && (dist2 <= Max_Step_Width))  )
                    {
                        return true;
                    }

                }

            }

        }
    }
    //Se não encontrou uma sequencia, retorna false
    return false;

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




void diane_octomap::DianeOctomap::StairDetection()
{
    //Início da marcacão de tempo de execucao da funcão
    time_t t_begin, t_end;

    t_begin = time(0);


    //***Obtendo os mínimos e os máximos (para obter o comprimento, a largura e a altura do espaco) --- Para Utilizar na Transformada de Hough
    vector<double> Space_Properties = GetSpaceProperties();



    //***Realizando a transformada de Hough e aplicando um filtro para obter somente os planos que receberam uma quantidade de votos dentro de um intervalo limite***
    //***O intervalo limite deve ser apdatativo (caso a resolucão do octomap mude, não saberemos qual será o intervalo limite correto)***
    vector<vector<double>> Hough_Planes = PlanesHoughTransform(Space_Properties.at(0), Space_Properties.at(1), Space_Properties.at(2));


    //Escrevendo nos arquivos os planos resultantes da Transformada de Hough (para plot no Matlab) --- Retirar para que diminuir o processamento;
    WriteFilteredPlanesToFile(Hough_Planes);



    //***Aglutinando (Merge) nos planos que são praticamente coplanares (inicialmente só utilizamos merge para planos que possuem o mesmo theta e o mesmo phi)***
    //***Assim, o merge foi aplicado para planos que estavam muito próximos entre si***

    //Chamando o método recursivo que aglutina os planos aproximadamente coplanares
    vector<vector<double>> Merged_Planes = MergePlanes(Hough_Planes);


    cout << "Após o merge, existem " << Merged_Planes.size() << " planos." << endl << endl;


    //Escrevendo os planos aglutinados em um arquivo (para plot no Matlab);
    WriteMergedPlanesToFile(Merged_Planes);



    //***Agrupando os planos restantes por seu Theta e Phi***
    //***Conferir se desse modo, o Rho poderia ser identificado como a largura do degrau***
    vector<vector<vector<double>>> Grouped_Planes = GroupPlanesByThetaPhi(Merged_Planes);



    //***Utilizando um histograma como um "Filtro" e para identificar a distância entre planos que mais ocorre***
    //***Foi inserido um filtro nesse histograma para só pontuar distâncias que estejam dentro do padrão de escadas***
    vector<vector<vector<double>>> Histogram_Grouped_Planes = GenerateHistogram(Grouped_Planes);


    //***Para cada grupo, verifica se existe uma sequência de 3 planos que estejam com distâncias dentro dos limites encontrados no histograma***
    vector<vector<vector<double>>> Sequenced_Grouped_Planes = FilterGroups(Histogram_Grouped_Planes);



    //***Estruturando uma escada para cada grupo de planos ainda existente***
    //***Dado que buscamos os planos verticais da escada, cada plano deveria nos retornar um degrau***
    //Obtendo os candidatos à escada
    vector<Stair*> StairCandidatesDetected = StairCandidatesDetection(Sequenced_Grouped_Planes);

//    WriteStairCandidateToFile(StairCandidatesDetected.at(9));


    //***Retirando as folhas dos degraus de cada candidato à escada que estão fora do padrão de degraus
    CleanStairsLeafs(StairCandidatesDetected);





    //***Atualizando os candidatos à escada
    //Para cada escada, retira os degraus que não são válidos. Se o candidato à escada não possuir mais do que 3 degraus válidos, ele não será mais um candidato.
    vector<Stair*> NewStairCandidates = CleanStairSteps(StairCandidatesDetected);


    //Escrevendo as informacões das escadas em um arquivo (para plot no Matlab)
//    WriteStairCandidateToFile(NewStairCandidates.at(0));


    //***Modelando cada candidato à escada (obtendo o comprimento, largura e altura média dos degraus, a aresta inicial e os pontos de arestas referentes aos outros degraus)
    vector<Stair*> Modeled_Stairs = ModelStairs(NewStairCandidates);



    //Escrevendo em um arquivo a aresta, comprimento, largura e altura da 1a escada (para plot no Matlab)
//    WriteModeledStairPropertiesToFile(Modeled_Stairs.at(0));





    //Fim da marcacao de tempo de execucao
    t_end = time(0);
    time_t tempo = t_end - t_begin;

    cout << "Demorou " << tempo << " segundos." << endl << endl;

}



vector<double> diane_octomap::DianeOctomap::GetSpaceProperties()
{
    vector<double> Space_Properties;

    double min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;

    for(int i = 0; i < OccupiedLeafsInBBX.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = OccupiedLeafsInBBX.at(i);

        if(leaf.getX() < min_x)
        {
            min_x = leaf.getX();
        }
        if(leaf.getX() > max_x)
        {
            max_x = leaf.getX();
        }

        if(leaf.getY() < min_y)
        {
            min_y = leaf.getY();
        }
        if(leaf.getY() > max_y)
        {
            max_y = leaf.getY();
        }

        if(leaf.getZ() < min_z)
        {
            min_z = leaf.getZ();
        }
        if(leaf.getZ() > max_z)
        {
            max_z = leaf.getZ();
        }
    }

    double Space_Length = max_x - min_x;
    double Space_Width = max_y - min_y;
    double Space_Height = max_z - min_z;

    Space_Properties.push_back(Space_Length);
    Space_Properties.push_back(Space_Width);
    Space_Properties.push_back(Space_Height);

    return Space_Properties;

}




//Os dados dos centróides serão obtidos diretamente do vetor de folhas
vector<vector<double>> diane_octomap::DianeOctomap::PlanesHoughTransform(double length, double width, double height)
{
    cout << "Funcão PlanesHoughTransform inicializada." << endl << endl;

    //Definindo o Rho, Theta e Phi mínimos e máximos (dado que só estamos querendo detectar planos verticais, estabelecemos um phi mínimo e um máximo para reduzir o processamento)
    Rho_Min = 0;
    Rho_Max = ceil(sqrt(pow(length, 2) + pow(width, 2) + pow(height, 2)));
    Theta_Min = 0;
    Theta_Max = 360;
    Phi_Min = 70;
    Phi_Max = 110;

    Rho_Passo = 0.05;
    Theta_Passo = 5;
    Phi_Passo = 5;


    Rho_Num = (Rho_Max - Rho_Min)/Rho_Passo;
    Theta_Num = (Theta_Max - Theta_Min)/Theta_Passo;
    Phi_Num = (Phi_Max - Phi_Min)/Phi_Passo;


    cout << "(Rho_Min, Rho_Max, Theta_Min, Theta_Max, Phi_Min, Phi_Max): (" << Rho_Min << ", " << Rho_Max << ", " << Theta_Min << ", " << Theta_Max << ", " << Phi_Min << ", " << Phi_Max << ")" << endl;
    cout << "Rho_Num: " << Rho_Num << ", Theta_Num: " << Theta_Num << ", Phi_Num: " << Phi_Num << "." << endl;


    //Inicializando o Acumulador
    InitializeAccumulator();


    //Para cada voxel ocupada, chama a funcão de acumulacão para realizar a votacão
    for(int i = 0; i<OccupiedLeafsInBBX.size(); i++)
    {
        AccumulatePoint(OccupiedLeafsInBBX.at(i));

    }

    cout << endl << "Votacão finalizada" << endl;


    //Printando o estado do acumulador
//    PrintAccumulator();


    //Obtendo os planos que passaram pelo filtro
    vector<vector<double>> Filtered_Planes = GetFilteredPlanes();


    cout << "Existem " << Filtered_Planes.size() << " planos filtrados." << endl << endl;


    cout << "Funcão PlanesHoughTransform finalizada." << endl;


    return Filtered_Planes;


}


void diane_octomap::DianeOctomap::InitializeAccumulator()
{
    //Initializing the acumulator with 0's;
    for(int i=0; i < Rho_Num; i++)
    {
        vector<vector<int>> vector_thetas;

        for(int j=0; j < Theta_Num; j++)
        {
            vector<int> vector_phis;

            for(int k=0; k < Phi_Num; k++)
            {
                vector_phis.push_back(0);
            }

            vector_thetas.push_back(vector_phis);
        }

        Accumulator.push_back(vector_thetas);
    }

}


void diane_octomap::DianeOctomap::AccumulatePoint(OcTree::leaf_bbx_iterator leaf_point)
{
    //Invertendo a ordem da iteracão para diminuir a quantidade de cálculos
    for(unsigned int i = 0; i<Phi_Num; i++)
    {
        double phi = (Phi_Min + (i+0.5)*Phi_Passo)*(M_PI/180);


        //Garantindo que o phi não ultrapassará o seu limite máximo
        if(phi > (Phi_Max*M_PI/180))
        {
            phi = (Phi_Max*M_PI/180);
        }

        for(unsigned int j = 0; j<Theta_Num; j++)
        {
            double theta = (Theta_Min + (j+0.5)*Theta_Passo)*(M_PI/180);


            //Garantindo que o theta não ultrapassará o seu limite máximo
            if(theta > ((Theta_Max*M_PI)/180))
            {
                theta = ((Theta_Max*M_PI)/180);
            }

            //Definindo a normal do plano
            double n[3];
            n[0] = cos(theta)*sin(phi);
            n[1] = sin(theta)*sin(phi);
            n[2] = cos(phi);

            for(unsigned int k = 0; k<Rho_Num; k++)
            {
                double rho = (Rho_Min + (k+0.5)*Rho_Passo);

                //Calculando o Rho do plano que passa no ponto sendo avaliado com o Theta e o Phi avaliados;
                double point_rho = leaf_point.getX() * n[0] + leaf_point.getY() * n[1] + leaf_point.getZ() * n[2];

                //Se a distância entre os dois planos, que possuem a mesma angulacão theta e phi, estiver dentro da tolerância, o ponto vota nessa célula
                if (fabs(point_rho - rho) < Max_Planes_Distance)
                {
                    ((Accumulator.at(k)).at(j)).at(i) = ((Accumulator.at(k)).at(j)).at(i) + 1;
                }

            }

        }

    }

}


vector<vector<double>> diane_octomap::DianeOctomap::GetFilteredPlanes()
{
    //Armazenando no vetor as coordenadas polares do centróide da célula
    vector<vector<double>> Filtered_Planes;

    int count = 0;

    for(unsigned int i = 0; i<Phi_Num; i++)
    {
        double phi = (Phi_Min + (i+0.5)*Phi_Passo);

        if ((phi>= Filter_Phi_Min) && (phi<= Filter_Phi_Max))
        {
            for(unsigned int j = 0; j<Theta_Num; j++)
            {
                double theta = (Theta_Min + (j+0.5)*Theta_Passo);

                for(unsigned int k = 0; k<Rho_Num; k++)
                {
                    double rho = (Rho_Min + (k+0.5)*Rho_Passo);

                    double votes = ((Accumulator.at(k)).at(j)).at(i);

                    if ((votes >= Filter_Vote_Min) && (votes <= Filter_Vote_Max))
                    {
                        //Inclui no vetor filtrado
                        vector<double> valid_plane;
                        valid_plane.push_back(rho);
                        valid_plane.push_back(theta);
                        valid_plane.push_back(phi);
                        valid_plane.push_back(votes);

                        Filtered_Planes.push_back(valid_plane);
                        count++;
                    }

                }

            }

        }

    }

    cout << count << " planos passaram pelo filtro." << endl << endl;

    return Filtered_Planes;

}


vector<vector<double>> diane_octomap::DianeOctomap::MergePlanes(vector<vector<double>> Planes)
{
    //Input:
    //Planes é o vector com todos os vetores referentes aos Planos a serem verificados para aglutinacão.
    //Cada vector em Planes contém: RHO, THETA, PHI e Votos nesse plano

    //Output:
    //Merged_Planes é o vector resultado com todos os vetores de Planos após serem aglutinados.

    vector<vector<double>> Merged_Planes;

    vector<double> NewPlane;

    int i = 0, j = 0;


    bool merge_break = false;

//    cout << "Tamanho de Planes: " << Planes.size() << "." << endl;

    if(Planes.size() > 0)
    {
        //Para cada plano, verifica se cada um dos planos seguintes podem ser agrupados com o inicial
        for(i=0; i<Planes.size(); i++)
        {
            for(j=i+1; j<Planes.size(); j++)
            {
                if(CanMergePlanes(Planes.at(i), Planes.at(j)))
                {
                    //Se dois planos forem marcados como mergeable, executa o MergePlanes repassando o novo Planes (com o novo Plane e sem os dois antigos)
                    NewPlane = FitPlane(Planes.at(i), Planes.at(j));
                    merge_break = true;
                    break;
                }

            }

            if (merge_break == true)
            {
                break;
            }

        }

    }

    //Se ocorreu um merge_break, gera um novo conjunto de Planes (com o novo Plane e sem os dois antigos) e chama o MergePlanes
    if(merge_break)
    {
        vector<vector<double>> temp_Planes;

        temp_Planes.push_back(NewPlane);

        //Copiando os planos antigos para o novo vetor
        for(int ii=0; ii<i; ii++)
        {
            temp_Planes.push_back(Planes.at(ii));
        }
        for(int jj = i+1; jj<j; jj++)
        {
            temp_Planes.push_back(Planes.at(jj));
        }
        for(int kk = j+1; kk<Planes.size(); kk++)
        {
            temp_Planes.push_back(Planes.at(kk));
        }

//        cout << "Tamanho de temp_Planes: " << temp_Planes.size() << "." << endl;

        //Recursão para obter somente os planos aglutinados finais
        Merged_Planes = MergePlanes(temp_Planes);

    }
    else
    {
        //Se não existirem mais planos a serem aglutinados, retorna o vector que foi recebido
        Merged_Planes = Planes;
    }

    return Merged_Planes;

}


bool diane_octomap::DianeOctomap::CanMergePlanes(vector<double> PlaneA, vector<double> PlaneB)
{
    //Input:
    //PlaneA é o vector com informacões do primeiro plano sendo analisado, contendo RHO, THETA e PHI.
    //PlaneB é o vector com informacões do segundo plano sendo analisado, contendo RHO, THETA e PHI.

    //Output:
    //Bool Result indicando se os dois planos podem ser aglutinados ou não

    //delta_Rho é a tolerância em Rho para a aglutinacão entre os planos.
    //delta_Theta é a tolerância em Theta para a aglutinacão entre os planos.
    //delta_Phi é a tolerância em Phi para a aglutinacão entre os planos.


    bool Result = false;

    double RhoA = PlaneA.at(0);
    double RhoB = PlaneB.at(0);
    double ThetaA = PlaneA.at(1);
    double ThetaB = PlaneB.at(1);
    double PhiA = PlaneA.at(2);
    double PhiB = PlaneB.at(2);

    if ((abs(RhoA - RhoB) <= delta_Rho) && (abs(ThetaA - ThetaB) <= delta_Theta) && (abs(PhiA - PhiB) <= delta_Phi))
    {
        //Seria bom adicionar algum outro filtro fora os limites? Não tenho a informacão dos pontos presentes no plano
        Result = true;
    }

    return Result;

}


vector<double> diane_octomap::DianeOctomap::FitPlane(vector<double> PlaneA, vector<double> PlaneB)
{
    //Inicialmente, o plano resultante possuirá parâmetros RHO, THETA e PHI iguais às médias ponderadas dos parâmetros dos planos recebidos, onde o peso é a quantidade de votos.

    vector<double> Result_Plane;

    double Rho = ((PlaneA.at(0)*PlaneA.at(3) + PlaneB.at(0)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
    double Theta = ((PlaneA.at(1)*PlaneA.at(3) + PlaneB.at(1)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
    double Phi = ((PlaneA.at(2)*PlaneA.at(3) + PlaneB.at(2)*PlaneB.at(3))/(PlaneA.at(3) + PlaneB.at(3)));
    double Votes = (PlaneA.at(3) + PlaneB.at(3));



    Result_Plane.push_back(Rho);
    Result_Plane.push_back(Theta);
    Result_Plane.push_back(Phi);
    Result_Plane.push_back(Votes);


    return Result_Plane;
}


//Separando os planos em grupos de par (Theta, Phi)
vector<vector<vector<double>>> diane_octomap::DianeOctomap::GroupPlanesByThetaPhi(vector<vector<double>> Planes)
{
    vector<vector<vector<double>>> GroupedPlanes;

    //Separando os planos por seus ângulos Theta e Phi
    for(int i=0; i<Planes.size(); i++)
    {
        vector<double> Plane = Planes.at(i);

        double theta = Plane.at(1);
        double phi = Plane.at(2);

        bool Group_Found = false;

        //Verificando se os ângulos do Plano já fazem parte de um grupo
        for(int j=0; j<GroupedPlanes.size(); j++)
        {
            if((theta == ((GroupedPlanes.at(j)).at(0)).at(1)) && (phi == ((GroupedPlanes.at(j)).at(0)).at(2)))
            {
                GroupedPlanes.at(j).push_back(Plane);
                Group_Found = true;
                break;
            }

        }

        if(Group_Found == false)
        {
            //Se o grupo não foi encontrado, cria um novo grupo
            vector<vector<double>> NewGroupPlanes;
            NewGroupPlanes.push_back(Plane);

            GroupedPlanes.push_back(NewGroupPlanes);
        }

    }

    return GroupedPlanes;

}


vector<vector<vector<double>>> diane_octomap::DianeOctomap::GenerateHistogram(vector<vector<vector<double>>> Grouped_Planes)
{
    //Montagem do histograma de frequência de distância entre os planos em um grupo
    double Hist_Passo = 0.025;
    int Hist_Length = ceil((Rho_Max - Rho_Min))/Hist_Passo; //Quantidade de intervalos do histograma
    vector<int> Histogram(Hist_Length, 0);

    map<vector<vector<double>>, double> Planes_Dist_Map;

    //Para cada grupo de planos (theta e phi fixos), verifica as distâncias entre os planos e pontua no histograma
    for(int k=0; k<Grouped_Planes.size(); k++)
    {
        if((Grouped_Planes.at(k)).size() >= Min_Num_Steps)
        {
            for(int l=0; l<(Grouped_Planes.at(k)).size() - 1; l++)
            {
                vector<double> PlaneA = (Grouped_Planes.at(k)).at(l);
                vector<double> PlaneB;

                for(int m=l+1; m<(Grouped_Planes.at(k)).size(); m++)
                {
                    PlaneB = (Grouped_Planes.at(k)).at(m);

                    double Dist = abs(PlaneA.at(0) - PlaneB.at(0));

                    //Só pontua se a distância estiver dentro do padrão de degrau
                    if((Dist > Min_Step_Width) && (Dist < Max_Step_Width))
                    {
                        //Adicionando um ponto para o intervalo do histograma.
                        int index = floor(Dist/Hist_Passo);

                        Histogram.at(index) = Histogram.at(index) + 1;

                        //Adicionar em alguma estrutura os dois planos que geraram essa distância
                        vector<vector<double>> Planes_Key;
                        Planes_Key.push_back(PlaneA);
                        Planes_Key.push_back(PlaneB);

                        Planes_Dist_Map.insert(pair< vector<vector<double>>, double>(Planes_Key, Dist));

                    }

                }

            }

        }

    }

    int max_freq = 0;
    int max_index = 0;
    for (int n=0; n<Hist_Length; n++)
    {
        if (Histogram.at(n) > max_freq)
        {
            max_freq = Histogram.at(n);
            max_index = n;
        }
    }

    //Obtendo o intervalo de distância tolerado para selecionar os planos
    Histogram_Dist_Min = (max_index - 1) * Hist_Passo;
    Histogram_Dist_Max = (max_index + 2) * Hist_Passo;


    cout << "Maior pontuacão do histograma: " << max_freq << "." << endl << endl;
    cout << "Índice da maior pontuacão do histograma: " << max_index << "." << endl << endl;

    cout << "Distância mínima: " << Histogram_Dist_Min << "." << endl << endl;
    cout << "Distância máxima: " << Histogram_Dist_Max << "." << endl << endl;


    //Extraindo do map os planos que fazem parte de um par de planos com distância tolerada
    return ExtractValidPlanes(Planes_Dist_Map);

}


vector<vector<vector<double>>> diane_octomap::DianeOctomap::ExtractValidPlanes(map<vector<vector<double>>, double> Planes_Dist_Map)
{
    //Verificando se o par de planos possuem uma distância dentro da tolerância
    vector<vector<double>> Valid_Planes;

    vector<vector<vector<double>>> Grouped_Valid_Planes;


    for(map<vector<vector<double>>, double>::iterator it=Planes_Dist_Map.begin(); it!=Planes_Dist_Map.end(); it++)
    {
        if((it->second >= Histogram_Dist_Min) && (it->second <= Histogram_Dist_Max))
        {
            vector<vector<double>> Plane_Pair = it->first;

            //Inserindo em Valid_Planes o 1o plano se já não existir
            if(!(find(Valid_Planes.begin(), Valid_Planes.end(), Plane_Pair.at(0)) != Valid_Planes.end()))
            {
                Valid_Planes.push_back(Plane_Pair.at(0));
            }

            //Inserindo em Valid_Planes o 2o plano se já não existir
            if(!(find(Valid_Planes.begin(), Valid_Planes.end(), Plane_Pair.at(1)) != Valid_Planes.end()))
            {
                Valid_Planes.push_back(Plane_Pair.at(1));
            }

        }

    }

    cout << "Existem " << Valid_Planes.size() << " planos válidos." << endl << endl;


    //Agrupando novamente por Theta e Phi
    Grouped_Valid_Planes = GroupPlanesByThetaPhi(Valid_Planes);


    cout << "Existem " << Grouped_Valid_Planes.size() << " grupos de planos válidos." << endl << endl;


    return Grouped_Valid_Planes;

}


vector<vector<vector<double>>> diane_octomap::DianeOctomap::FilterGroups(vector<vector<vector<double>>> Groups_Planes)
{
    vector<vector<vector<double>>> Sequenced_Groups;

    //Filtrando os grupos de planos (se ele não possuir uma sequência, o grupo inteiro é removido)
    for(int i=0; i<Groups_Planes.size(); i++)
    {
        vector<vector<double>> Group = Groups_Planes.at(i);

        if(VerifySequence(Group))
        {
            Sequenced_Groups.push_back(Group);
        }

    }

    return Sequenced_Groups;
}


bool diane_octomap::DianeOctomap::VerifySequence(vector<vector<double>> Group_Planes)
{
    vector<double> Rhos;

    for(int j=0; j<Group_Planes.size(); j++)
    {
        Rhos.push_back((Group_Planes.at(j)).at(0));
    }


    sort(Rhos.begin(), Rhos.end());

    for(int k=0; k<Rhos.size()-2; k++)
    {
        for(int l=k+1; l<Rhos.size()-1; l++)
        {

            for(int m=l+1; m<Rhos.size(); m++)
            {
                double dist1 = fabs(Rhos.at(k) - Rhos.at(l));
                double dist2 = fabs(Rhos.at(l) - Rhos.at(m));

                if(((dist1 >= Histogram_Dist_Min) && (dist1 <= Histogram_Dist_Max)) && ((dist2 >= Histogram_Dist_Min) && (dist2 <= Histogram_Dist_Max)) )
                {
                    return true;
                }

            }

        }

    }

    //Se não encontrou uma sequencia, retorna false
    return false;

}


vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::StairCandidatesDetection(vector<vector<vector<double>>> Grouped_Planes)
{
    vector<Stair*> StairCandidatesDetected;

    //Obtendo os candidatos à escada
    for(int i=0; i<Grouped_Planes.size(); i++)
    {
        //Se no grupo de planos existirem planos suficientes para detectar uma escada, obtém a candidata da escada
        if(Grouped_Planes.at(i).size() >= Min_Num_Steps)
        {
            Stair* NewStairCandidate = ObtainStairCandidateFromGroup(Grouped_Planes.at(i));

            if((NewStairCandidate->Steps.size() != 0))
            {
                StairCandidatesDetected.push_back(NewStairCandidate);

            }

        }

    }

    return StairCandidatesDetected;

}



diane_octomap::Stair* diane_octomap::DianeOctomap::ObtainStairCandidateFromGroup(vector<vector<double>> Group_Planes)
{
    Stair* StairCandidate = new Stair();

    //Para cada plano do grupo, armazena os pontos que fazem parte/estão à uma distância tolerável deste plano
    for(int i=0; i<Group_Planes.size(); i++)
    {
        Step* NewStep = new Step();

        vector<double> Plane = Group_Planes.at(i);
        double rho = Plane.at(0);
        double theta = Plane.at(1) * (M_PI/180);
        double phi = Plane.at(2) * (M_PI/180);

        //Definindo a normal do plano
        double n[3];
        n[0] = cos(theta)*sin(phi);
        n[1] = sin(theta)*sin(phi);
        n[2] = cos(phi);

        NewStep->Step_Plane = Plane;

        //Para cada folha, verifica se a centróide está próxima do plano
        for(int j=0; j<OccupiedLeafsInBBX.size(); j++)
        {
            OcTree::leaf_bbx_iterator leaf = OccupiedLeafsInBBX.at(j);

            //Calculando o Rho do plano que passa no ponto sendo avaliado com o Theta e o Phi do plano avaliado;
            double point_rho = leaf.getX() * n[0] + leaf.getY() * n[1] + leaf.getZ() * n[2];

            //Se a distância entre os dois planos, que possuem a mesma angulacão theta e phi, estiver dentro da tolerância, o ponto vota nessa célula
            if (fabs(point_rho - rho) < Max_Planes_Distance)
            {
                NewStep->Leafs_In_Step.push_back(leaf);
            }

        }

        //Inserindo o Degrau referente ao plano no candidato à escada
        StairCandidate->Steps.push_back(NewStep);

    }

    return StairCandidate;

}


void diane_octomap::DianeOctomap::CleanStairsLeafs(vector<Stair*>& Detected_Stair_Candidates)
{
    for(int i=0; i<Detected_Stair_Candidates.size(); i++)
    {
        Stair* stair = Detected_Stair_Candidates.at(i);

        for(int j=0; j<stair->Steps.size(); j++)
        {
            Step* step = stair->Steps.at(j);

            //Para cada degrau, filtra as colunas que possuem alturas fora do padrão de degraus
            step->StepHeightFilter();

            //Aplicando o filtro de histograma para o degrau (Degraus que não tiverem um padrão nas alturas do seu degrau serão considerados inválidos)
            step->StepHeightHistogram();

        }

    }

}


vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::CleanStairSteps(vector<diane_octomap::Stair*>& Detected_Stair_Candidates)
{
    vector<Stair*> NewStairCandidates;

    for(int i=0; i<Detected_Stair_Candidates.size(); i++)
    {
        Stair* stair = Detected_Stair_Candidates.at(i);

        vector<Step*> NewSteps;
        NewSteps.clear();

        for(int j=0; j<stair->Steps.size(); j++)
        {
            Step* step = stair->Steps.at(j);

            //Verifica se o degrau possui folhas (se não tiver, não é um degrau válido)
            if(step->Leafs_In_Step.size() > 0)
            {
                NewSteps.push_back(step);
            }
        }

        //Se a quantidade de degraus válidos for o suficiente para formar uma escada, essa escada ainda é uma candidata
        if(NewSteps.size() > Min_Num_Steps)
        {
            stair->Steps = NewSteps;
            stair->Num_Steps = NewSteps.size();

            NewStairCandidates.push_back(stair);
        }

    }

    return NewStairCandidates;

}


vector<diane_octomap::Stair*> diane_octomap::DianeOctomap::ModelStairs(vector<diane_octomap::Stair*>& Stair_Candidates)
{
    vector<Stair*> Modeled_Stairs;

    for(int i=0; i<Stair_Candidates.size(); i++)
    {
        Stair* stair = Stair_Candidates.at(i);

        //Passando para a escada todos as folhas presentes nos seus degraus e calculando os X, Y e Z mínimos e máximos
        stair->ExtractLeafsFromSteps();
        stair->CalculateStairProperties();


        //Ordenando os degraus da escada de acordo com a posicão média Z de cada degrau, em ordem crescente
        stair->SortSteps();


        //Modelando a escada
        stair->ModelStair(Octree_Resolution);


        Modeled_Stairs.push_back(stair);

    }

    return Modeled_Stairs;

}


void diane_octomap::DianeOctomap::PrintAccumulator()
{
    int count = 0;

    for(unsigned int i = 0; i<Phi_Num; i++)
    {
        double phi1 = (Phi_Min + (i)*Phi_Passo);
        double phi2 = (Phi_Min + (i+1)*Phi_Passo);

        if ((phi1 >= Filter_Phi_Min) && (phi2 <= Filter_Phi_Max))
        {
            for(unsigned int j = 0; j<Theta_Num; j++)
            {
                double theta1 = (Theta_Min + (j)*Theta_Passo);
                double theta2 = (Theta_Min + (j+1)*Theta_Passo);

                for(unsigned int k = 0; k<Rho_Num; k++)
                {
                    double rho1 = (Rho_Min + (k)*Rho_Passo);
                    double rho2 = (Rho_Min + (k+1)*Rho_Passo);

                    double votes = 0;

                    votes = ((Accumulator.at(k)).at(j)).at(i);

                    if ((votes > Filter_Vote_Min) && (votes < Filter_Vote_Max))
                    {

                        cout << "(rho1, rho2, theta1, theta2, phi1, phi2, Total Votes): (" << rho1 << ", " << rho2 << ", " << theta1 << ", " << theta2 << ", " << phi1 << ", " << phi2 << ", " << votes << ")" << endl;
                        cout << endl;

                        count++;

                    }

                }

            }

            cout << endl << endl;

        }

    }

    cout << "Quantidade de planos encontrados: " << count << "." << endl;

}


void diane_octomap::DianeOctomap::WriteFilteredPlanesToFile(vector<vector<double>> Filtered_Planes)
{
    ofstream filteredfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/filtered_planes.txt");

    filteredfile << "A = [";

    for(int i=0; i<Filtered_Planes.size(); i++)
    {
        vector<double> Filtered_Plane = Filtered_Planes.at(i);

        filteredfile << Filtered_Plane.at(0) << " " << Filtered_Plane.at(1) << " " << Filtered_Plane.at(2) << ";";
    }


    filteredfile << "]" << endl;
    filteredfile.close();

}


void diane_octomap::DianeOctomap::WriteMergedPlanesToFile(vector<vector<double>> Merged_Planes)
{
    ofstream mergedfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/merged_planes.txt");

    mergedfile << "A = [";

    for(int i=0; i<Merged_Planes.size(); i++)
    {
        vector<double> Merged_Plane = Merged_Planes.at(i);

        mergedfile << Merged_Plane.at(0) << " " << Merged_Plane.at(1) << " " << Merged_Plane.at(2) << ";";
    }


    mergedfile << "]" << endl;
    mergedfile.close();

}


void diane_octomap::DianeOctomap::WriteStairCandidateToFile(diane_octomap::Stair* Stair)
{
    ofstream stairfile("/home/derekchan/Dropbox/Projeto Final/Arquivos/stair_candidate.txt");

    stairfile << "A = [";

    for(int i=0; i<Stair->Steps.size(); i++)
    {
        Step* step = Stair->Steps.at(i);

        for(int j=0; j<step->Leafs_In_Step.size(); j++)
        {
            OcTree::leaf_bbx_iterator leaf = step->Leafs_In_Step.at(j);

            stairfile << leaf.getX() << ", " << leaf.getY() << ", " << leaf.getZ() << ";";
        }

    }


    stairfile << "];" << endl;
    stairfile.close();


}


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

    //Ordenando os degraus em ordem crescente da média de Z
    for(int i=0; i<Steps.size(); i++)
    {
        Mean_Index_Map.insert(pair<double, int>(Steps.at(i)->Step_Mean_Z, i));
    }

    for(map<double, int>::iterator itr = Mean_Index_Map.begin(); itr != Mean_Index_Map.end(); itr++)
    {
        //Adicionando o degrau no vetor ordenado
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

            //Verificando se a folha ja faz parte das folhas da escada. Se não fizer, atualiza as informacões da escada
            if((find(Leafs_In_Stair.begin(), Leafs_In_Stair.end(), leaf) != Leafs_In_Stair.end()) == false)
            {
                Leafs_In_Stair.push_back(leaf);
            }

        }

    }

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



    //Calulando o centróide do primeiro degrau
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




diane_octomap::Stair::~Stair()
{

}


///******Métodos da classe Line*******


diane_octomap::Line::Line()
{
    Line_Rho = 0;
    Line_Theta = 0;
    Line_Votes = 0;
    Line_Z = 0;

    min_X = 1000;
    max_X = -1000;
    min_Z = 1000;
    max_Z = -1000;
}


//Funcão que observa as folhas presentes na linha e atualiza os limites de X e de Z da linha
void diane_octomap::Line::UpdateLimits()
{
    for(int i=0; i<Leafs_In_Line.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf = Leafs_In_Line.at(i);

        //Atualizando os limites de X
        if(min_X > leaf.getX())
        {
            min_X = leaf.getX();
        }
        if(max_X < leaf.getX())
        {
            max_X = leaf.getX();
        }

        //Atualizando os limites de Z
        if(min_Z > leaf.getZ())
        {
            min_Z = leaf.getZ();
        }
        if(max_Z < leaf.getZ())
        {
            max_Z = leaf.getZ();
        }
    }

}

void diane_octomap::Line::sortLeafs()
{
    bool cont = true;
    while(cont)
    {
        cont=false;
        for (int i = 0;i < Leafs_In_Line.size()-1; i++)
        {
            if(Leafs_In_Line.at(i).getX() > Leafs_In_Line.at(i+1).getX())
            {
                OcTree::leaf_bbx_iterator leaf = Leafs_In_Line.at(i);
                Leafs_In_Line.at(i) = Leafs_In_Line.at(i+1);
                Leafs_In_Line.at(i+1) = leaf;
                cont=true;
            }
        }
    }
}

diane_octomap::Line::~Line()
{

}
