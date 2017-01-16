// diane_octomap.cpp

#include <iostream>

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
    string otFileName = "/home/derekchan/catkin_workspace/src/sistema_observacao/files/MapFiles/Octree/Escada_Kinect_5.ot";

    AbstractOcTree* abs_tree = AbstractOcTree::read(otFileName);
    if(abs_tree) // read error returns NULL
    {
        octree = dynamic_cast<OcTree*>(abs_tree);

        if (octree) // cast succeeds if correct type
        {
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
    //Tentando acessar os dados das folhas
    for(int i = 0; i < OccupiedLeafsInBBX.size(); i++)
    {
        OcTree::leaf_bbx_iterator leaf_it = OccupiedLeafsInBBX.at(i);

        string Result = "Folha ";
        ostringstream convert_i;
        ostringstream convert_x;
        ostringstream convert_y;
        ostringstream convert_z;

        convert_i << i + 1;
        Result = convert_i.str() + ": x = ";
        convert_x << leaf_it.getX();
        Result = Result + convert_x.str() + "; y = ";
        convert_y << leaf_it.getY();
        Result = Result + convert_y.str() + "; z = ";
        convert_z << leaf_it.getZ();
        Result = Result + convert_z.str();

        cout << Result << endl;

    }

}


diane_octomap::DianeOctomap::~DianeOctomap()
{
    StopInternalCycle();
}
