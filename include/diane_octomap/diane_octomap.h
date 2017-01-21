/*!
 * \file diane_octomap.h
 */



#ifndef DIANE_DIANE_OCTOMAP_H
#define DIANE_DIANE_OCTOMAP_H


#include <vector>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>


#include <boost/thread.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>


using std::vector;

using namespace octomap;
using namespace std;

namespace diane_octomap {


struct maxcompare
{
    bool operator()(int* ip1, int* ip2) const
    {
        return (ip1[0] >ip2[0]);
    }
};



/*!
 * \class DianeOctomap
 * \brief The DianeOctomap class
 */
class DianeOctomap
{
    /// Pointer used to control the thread responsible for the internal
    ///     cycle.
    boost::thread* internalThread;

    /// Variable used to tell the controller to stop or not.
    bool stop;


    /*!
     * \brief InternalThreadFunction calls the function
     *      DianeOctomap::InternalCycleProcedure().
     */
    static void InternalThreadFunction(DianeOctomap* diane_octomap);

    /*!
     * \brief InternalCycleProcedure calls the function
     *      DianeOctomap:InternalCycle() from in each period of time
     *      specified.
     */
    virtual void InternalCycleProcedure();


protected:
    /// Mutex used to control the internal cycle thread.
    boost::mutex mutStartStop;

    //OcTree referente ao mapa estático, criada à partir do arquivo .ot inicial.
    OcTree* octree;
    OcTree* octreeFromMsg;

    vector<OcTree::leaf_bbx_iterator> OccupiedLeafsInBBX;

    //Variáveis referentes à Transformada de Hough
    double Rho_Min;
    double Rho_Max;
    double Theta_Min;   //Em graus
    double Theta_Max;   //Em graus
    double Phi_Min;     //Em graus
    double Phi_Max;     //Em graus

    double Rho_Passo;   //Incremento em Rho
    double Theta_Passo; //Incremento em Theta
    double Phi_Passo;   //Incremento em Phi

    double Rho_Num;
    double Theta_Num;
    double Phi_Num;
    double Max_Distance;

    vector<vector<vector<int>>> Accumulator;

    //Variáveis para filtrar os planos a serem extraídos
    double Filter_Phi_Min = 85;
    double Filter_Phi_Max = 96;
    double Filter_Vote_Min = 250;
    double Filter_Vote_Max = 400;


public:
    DianeOctomap();


    void onInit();


    /*!
     * \brief StartInternalCycle starts the internal cycle of the
     *      controller.
     */
    void StartInternalCycle();


    /*!
     * \brief StopInternalCyle waits for the internal cycle to end then
     *      stops the controller.
     */
    void StopInternalCycle();


    //Método para obtencão da octree à partir de um arquivo
    void GenerateOcTreeFromFile();


    //Métodos utilizados para a identificar uma escada dada uma OcTree;
    void GetOccupiedLeafsOfBBX(OcTree* octree);


    //A identificacão da escada será sobre o vetor de folhas contidas dentro do Bounding Box
    void StairDetection();


    //Metodos para Transformada de Hough
    void PlanesHoughTransform(double length, double width, double height);


    void InitializeAccumulator();


    void AccumulatePoint(OcTree::leaf_bbx_iterator p);


    vector<vector<double>> getFilteredPlanes();


    void MergePlanes(vector<vector<double>> Planes);


    void PrintAccumulator();


    /**
     * Returns a sorted list of the all cells in the accumulator.
     * @return a sorted multiset containing the cells as (counter, rho_index, theta_index, phi_index)
     */
    multiset<int*, maxcompare>* getMax();


    virtual ~DianeOctomap();
};


}

#endif
