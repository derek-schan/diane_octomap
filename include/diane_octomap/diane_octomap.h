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


//Estrutura que armazenará as folhas referentes à esse degrau e as informacões de um degrau (comprimento, largura e altura)
class Step
{
protected:

public:
    vector<OcTree::leaf_bbx_iterator> Leafs_In_Step;
    double Step_Length;
    double Step_Width;
    double Step_Height;

    double Step_Min_Z = 1000;
    double Step_Max_Z = -1000;
    double Step_Mean_Z = 0;

    vector<double> Step_Plane;

    Step();

    void CalculateStepProperties();

    virtual ~Step();

};


//Estrutura que armazenará informacões de uma escada (comprimento total, largura total e altura total).
class Stair
{
protected:

public:
    vector<Step*> Steps;

    vector<OcTree::leaf_bbx_iterator> Leafs_In_Stair;

    double Total_Length;
    double Total_Width;
    double Total_Height;

    double Min_X = 1000;
    double Max_X = -1000;
    double Min_Y = 1000;
    double Max_Y = -1000;
    double Min_Z = 1000;
    double Max_Z = -1000;


    int Num_Steps = 0;
    double Step_Length = 0;
    double Step_Width = 0;
    double Step_Height = 0;

    //(Xi,Yi) e (Xf,Yf)
    vector<double> Aresta;


    Stair();

    //Método referente à ordenacão dos degraus
    void SortSteps();

    //Método para obter as folhas presentes nos degraus
    void ExtractLeafsFromSteps();


    //Método obtendo os X, Y e Z mínimos e máximos da escada
    void CalculateStairProperties();


    virtual ~Stair();

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

    double Octree_Resolution;

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

    //Diferenca máxima entre o Rho do ponto e o Rho da célula para ser considerado como um voto
    double Max_Planes_Distance = 0.1;

    vector<vector<vector<int>>> Accumulator;

    //Variáveis para filtrar os planos a serem extraídos
    double Filter_Phi_Min = 85;
    double Filter_Phi_Max = 96;
    double Filter_Vote_Min = 250;
    double Filter_Vote_Max = 400;

    //Definindo as tolerâncias para o merge dos planos
    double delta_Rho = 0.1;
    double delta_Theta = 0;
    double delta_Phi = 0;

    //Definindo variáveis resultantes do histograma
    double Histogram_Dist_Min;
    double Histogram_Dist_Max;

    //Definindo características da escada
    int Min_Num_Steps = 3;
    double Min_Step_Width = 0.25;
    double Max_Step_Width = 0.35;
    double Min_Step_Height = 0.09;
    double Max_Step_Height = 0.20;


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
    vector<vector<double>> PlanesHoughTransform(double length, double width, double height);


    void InitializeAccumulator();


    void AccumulatePoint(OcTree::leaf_bbx_iterator p);


    vector<vector<double>> GetFilteredPlanes();



    //Métodos para merge dos planos que são praticamente coplanares
    vector<vector<double>> MergePlanes(vector<vector<double>> Planes);


    bool CanMergePlanes(vector<double> PlaneA, vector<double> PlaneB);


    vector<double> FitPlane(vector<double> PlaneA, vector<double> PlaneB);



    //Método para agrupar os planos pelo (Theta e Phi)
    vector<vector<vector<double>>> GroupPlanesByThetaPhi(vector<vector<double>> Planes);



    //Métodos referentes à criacão do Histograma (Necessário? Está funcionando somente como um filtro)
    vector<vector<vector<double>>> GenerateHistogram(vector<vector<vector<double>>> SameAnglesPlanes);


    //Metodo para extracao dos planos que passaram pelo filtro do histograma
    vector<vector<vector<double>>> ExtractValidPlanes(map<vector<vector<double>>, double> Planes_Dist_Map);


    //Método de filtro do conjunto de planos
    vector<vector<vector<double>>> FilterGroups(vector<vector<vector<double>>> Groups_Planes);


    //Método que verifica se há uma sequência
    bool VerifySequence(vector<vector<double>> Group_Planes);



    vector<Stair*> StairCandidatesDetection(vector<vector<vector<double>>> Grouped_Planes);


    //Métodos referentes à aplicacão dos planos encontrados no octomap.
    Stair* ObtainStairCandidateFromGroup(vector<vector<double>> Group_Planes);


    void StepHistogram(Step* step);



    //Métodos referentes à couts e à escrita em arquivos
    void PrintAccumulator();


    void WriteFilteredPlanesToFile(vector<vector<double>> Filtered_Planes);


    void WriteMergedPlanesToFile(vector<vector<double>> Merged_Planes);


    void WriteStairToFile(diane_octomap::Stair* Stair);


    virtual ~DianeOctomap();
};


}

#endif
