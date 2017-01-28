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
    DianeOctomap::StairDetection();

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
    string otFileName = "/home/derekchan/catkin_workspace/src/diane_octomap/files/MapFiles/Octree/Escada_Kinect_5.ot";

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
    point3d min;
    min.x() = -0.70;
    min.y() = 0;
    min.z() = 0;

    point3d max;
    max.x() = 0.65;
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


    //***Retirando as folhas dos degraus de cada escada que estão fora do padrão de degraus
    for(int i=0; i<StairCandidatesDetected.size(); i++)
    {
        Stair* stair = StairCandidatesDetected.at(i);

        for(int j=0; j<stair->Steps.size(); j++)
        {
            Step* step = stair->Steps.at(j);

            //Para cada degrau, filtra as colunas que possuem alturas fora do padrão de degraus
            step->Step_Height_Filter();

            //Aplicando o filtro de histograma para o degrau (Degraus que não tiverem um padrão nas alturas do seu degrau serão considerados inválidos)
            step->StepHeightHistogram();

        }

    }




    //Escrevendo as informacões das escadas em um arquivo (para plot no Matlab)
    WriteStairCandidateToFile(StairCandidatesDetected.at(7));





    //Para cada escada, retira os degraus que não são válidos. Se o candidato à escada não possuir mais do que 3 degraus válidos, ele não será mais um candidato.
    vector<Stair*> NewStairCandidates;

    for(int i=0; i<StairCandidatesDetected.size(); i++)
    {
        Stair* stair = StairCandidatesDetected.at(i);

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

    //Atualizando os candidatos à escada
    StairCandidatesDetected = NewStairCandidates;


    //***Modelando cada candidato à escada (obtendo o comprimento, largura e altura média dos degraus, e a aresta inicial)
    for(int i=0; i<StairCandidatesDetected.size(); i++)
    {
        Stair* stair = StairCandidatesDetected.at(i);

        //Passando para a escada todos as folhas presentes nos seus degraus e calculando os X, Y e Z mínimos e máximos
        stair->ExtractLeafsFromSteps();
        stair->CalculateStairProperties();


        //Ordenando os degraus da escada de acordo com a posicão média Z de cada degrau, em ordem crescente
        stair->SortSteps();


        //Modelando a escada
        stair->ModelStair(Octree_Resolution);

    }


    //Escrevendo em um arquivo a aresta, comprimento, largura e altura da 1a escada (para plot no Matlab)
    WriteModeledStairPropertiesToFile(StairCandidatesDetected.at(1));



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


void diane_octomap::Step::Step_Height_Filter()
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

    bool sequence_found = false;

    if (index_vote.size() > 3)
    {
        for(int i=0; i<index_vote.size() - 3; i++)
        {
            if ((index_vote[i]==index_vote[i+1]-1) && (index_vote[i]==index_vote[i+2]-2) && (index_vote[i]==index_vote[i+3]-3))
            {
                sequence_found = true;
                double minZ = index_vote[i]*0.05;
                double maxZ = (index_vote[i+3] + 1)*0.05;

                vector <OcTree::leaf_bbx_iterator> newLeafs;

                for (int j=0;j<Leafs_In_Step.size();j++)
                {
                    if((Leafs_In_Step.at(j).getZ()>minZ) && (Leafs_In_Step.at(j).getZ()<maxZ))
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
        Leafs_In_Step.clear();
    }


}



diane_octomap::Step::~Step()
{

}




///******Métodos da classe Stair*******


diane_octomap::Stair::Stair()
{
    Min_X = 1000;
    Max_X = -1000;
    Min_Y = 1000;
    Max_Y = -1000;
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

        if(Min_X > leaf.getX())
        {
            Min_X = leaf.getX();
        }
        if(Max_X < leaf.getX())
        {
            Max_X = leaf.getX();
        }

        if(Min_Y > leaf.getY())
        {
            Min_Y = leaf.getY();
        }
        if(Max_Y < leaf.getY())
        {
            Max_Y = leaf.getY();
        }

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

}




diane_octomap::Stair::~Stair()
{

}

