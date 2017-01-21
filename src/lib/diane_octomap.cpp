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
    //Obtendo os mínimos e os máximos (para obter o comprimento, a largura e a altura)
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

    double length = max_x - min_x;
    double width = max_y - min_y;
    double height = max_z - min_z;

    PlanesHoughTransform(length, width, height);


}


//Os dados dos centróides serão obtidos diretamente do vetor de folhas
void diane_octomap::DianeOctomap::PlanesHoughTransform(double length, double width, double height)
{
    cout << "Funcão PlanesHoughTransform inicializada." << endl << endl;

    time_t t_begin, t_end;

    t_begin = time(0);

    //Definindo o Rho, Theta e Phi mínimos e máximos (dado que só estamos querendo detectar planos verticais, estabelecemos um phi mínimo e um máximo para reduzir o processamento)
    Rho_Min = 0;
    Rho_Max = round(sqrt(pow(length, 2) + pow(width, 2) + pow(height, 2)));
    Theta_Min = 0;
    Theta_Max = 360;
    Phi_Min = 70;
    Phi_Max = 110;

    Rho_Passo = 0.05;
    Theta_Passo = 5;
    Phi_Passo = 5;

    Max_Distance = 0.1;

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

    t_end = time(0);
    time_t tempo = t_end - t_begin;


    //Obtendo os planos que passaram pelo filtro
    vector<vector<double>> Filtered_Planes = getFilteredPlanes();



    //Printando o resultado do acumulador
    PrintAccumulator();


    cout << "Demorou " << tempo << " segundos." << endl;


    cout << endl << "Funcão PlanesHoughTransform finalizada." << endl;
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
                if (fabs(point_rho - rho) < Max_Distance)
                {
                    ((Accumulator.at(k)).at(j)).at(i) = ((Accumulator.at(k)).at(j)).at(i) + 1;
                }

            }

        }

    }

}


vector<vector<double>> diane_octomap::DianeOctomap::getFilteredPlanes()
{
    //Armazenando no vetor as coordenadas polares do centróide da célula
    vector<vector<double>> Filtered_Planes;

    int count = 0;

    for(unsigned int i = 0; i<Phi_Num; i++)
    {
        double phi = (Phi_Min + (i+0.5)*Phi_Passo);

        if ((phi>= Filter_Phi_Min) && (phi>= Filter_Phi_Max))
        {
            for(unsigned int j = 0; j<Theta_Num; j++)
            {
                double theta = (Theta_Min + (j+0.5)*Theta_Passo);

                for(unsigned int k = 0; k<Rho_Num; k++)
                {
                    double rho = (Rho_Min + (k+0.5)*Rho_Passo);

                    double votes = ((Accumulator.at(k)).at(j)).at(i);

                    if ((votes > Filter_Vote_Min) && (votes < Filter_Vote_Max))
                    {
                        //Inclui no vetor filtrado
                        vector<double> valid_plane;
                        valid_plane.push_back(rho);
                        valid_plane.push_back(theta);
                        valid_plane.push_back(phi);

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


void diane_octomap::DianeOctomap::MergePlanes(vector<vector<double>> Planes)
{
    //Método referente ao Merge dos Planos filtrados


}


void diane_octomap::DianeOctomap::PrintAccumulator()
{
    ofstream myfile("/home/derekchan/output.txt");

    myfile << "[";

    int count = 0;

    for(unsigned int i = 0; i<Phi_Num; i++)
    {
        double phi1 = (Phi_Min + (i)*Phi_Passo);
        double phi2 = (Phi_Min + (i+1)*Phi_Passo);

        double phi = (Phi_Min + (i+0.5)*Phi_Passo);


        if ((phi1 >= Filter_Phi_Min) && (phi2 <= Filter_Phi_Max))
        {
            for(unsigned int j = 0; j<Theta_Num; j++)
            {
                double theta1 = (Theta_Min + (j)*Theta_Passo);
                double theta2 = (Theta_Min + (j+1)*Theta_Passo);

                double theta = (Theta_Min + (j+0.5)*Theta_Passo);


                for(unsigned int k = 0; k<Rho_Num; k++)
                {
                    double rho1 = (Rho_Min + (k)*Rho_Passo);
                    double rho2 = (Rho_Min + (k+1)*Rho_Passo);

                    double rho = (Rho_Min + (k+0.5)*Rho_Passo);


                    double votes = 0;

                    votes = ((Accumulator.at(k)).at(j)).at(i);

                    if ((votes > Filter_Vote_Min) && (votes < Filter_Vote_Max))
                    {

                        myfile << rho << " " << theta << " " << phi << ";";

//                        myfile << "(rho1, rho2, theta1, theta2, phi1, phi2, Total Votes): (" << rho1 << ", " << rho2 << ", " << theta1 << ", " << theta2 << ", " << phi1 << ", " << phi2 << ", " << votes << ")" << endl;
//                        myfile << endl;

                        cout << "(rho1, rho2, theta1, theta2, phi1, phi2, Total Votes): (" << rho1 << ", " << rho2 << ", " << theta1 << ", " << theta2 << ", " << phi1 << ", " << phi2 << ", " << votes << ")" << endl;
                        cout << endl;

                        count++;

                    }

                }

            }

//            myfile << endl << endl;
            cout << endl << endl;

        }

    }

    myfile << "]" << endl;
    myfile.close();


//    cout << "Iniciando print das células com votos ordenados" << endl;

//    multiset<int*, diane_octomap::maxcompare>* sorted_list = getMax();

//    multiset<int*, maxcompare>::iterator it = sorted_list->begin();
//    int inferior_threshold = (1000);
//    int superior_threshold = (15000);
////    unsigned int stop = (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();

//    while((it != sorted_list->end()) && ((*it)[0] > inferior_threshold) && ((*it)[0] < superior_threshold))
//    {
//        double rho_value = ((*it)[1]) * Rho_Max/ Rho_Num;
//        double theta_value = ((*it)[2] * 2*180) / Theta_Num;
//        double phi_value = ((*it)[3] * 180) / (Phi_Num * 0.99999999);

//        cout << "(Rho, Theta, Phi, Votos): " << "(" << rho_value << ", " << theta_value << ", " << phi_value << ", " << (*it)[0] << ")\n" << endl;
//        it++;
//    }

    cout << "Quantidade de planos encontrados: " << count << "." << endl;


}



multiset<int*, diane_octomap::maxcompare>* diane_octomap::DianeOctomap::getMax()
{
    //Neste multiset, as listas referentes às células são ordenadas pelo struct maxcompare, ordenando pela quantidade de votos
    multiset<int*, maxcompare>* maxlist = new multiset<int*, maxcompare>();

    for(unsigned int i = 0; i < Rho_Num; i++)
    {
        for(unsigned int j = 0; j < Theta_Num; j++)
        {
            for(unsigned int k = 0; k < Phi_Num; k++)
            {
                int* temporary  = new int[4];
                temporary[0] = ((Accumulator.at(i)).at(j)).at(k);
                temporary[1] = i;
                temporary[2] = j;
                temporary[3] = k;
                maxlist->insert(temporary);
            }
        }
    }

    return maxlist;
}


diane_octomap::DianeOctomap::~DianeOctomap()
{
    StopInternalCycle();
}
