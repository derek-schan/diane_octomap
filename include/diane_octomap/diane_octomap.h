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

#include <eigen3/Eigen/Core>

#include "igl/sort.h"
#include "igl/slice.h"


using std::vector;
using namespace Eigen;

using namespace octomap;
using namespace std;


namespace diane_octomap {


class Line
{
protected:

public:
    vector<OcTree::leaf_bbx_iterator> Leafs_In_Line;
    MatrixXf Leafs_Of_Line;

    double Line_Rho;
    double Line_Theta;
    double Line_Votes;
    double Line_Z;

    //Variáveis que serão utilizadas quando forem agrupadas
    double min_X;
    double max_X;
    double min_Z;
    double max_Z;


    Line();

    void sortLeafs();

    void UpdateLimits();


    void SortLeafMatrixByX();

    void SortLeafMatrixByZ();

    void UpdateLimitsWithMatrix();


    virtual ~Line();

};


//Estrutura que armazenará as folhas referentes à esse degrau e as informacões de um degrau (comprimento, largura e altura)
class Step
{
protected:

public:
    vector<OcTree::leaf_bbx_iterator> Leafs_In_Step;
    double Step_Length;
    double Step_Width;
    double Step_Height;

    double Step_Min_Z;
    double Step_Max_Z;
    double Step_Mean_Z;

    vector<double> Step_Plane;

    Line* Step_Line;

    //Características padrões
    double Min_Step_Height;
    double Max_Step_Height;


    Step();

    void CalculateStepProperties();


    //Filtrando as colunas de folhas que estão fora dos limites padrões de altura para degraus
    void StepHeightFilter();


    void StepHeightHistogram();


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

    double Min_Z;
    double Max_Z;


    int Num_Steps;
    double Step_Length;
    double Step_Width;
    double Step_Height;


    //(Xi,Yi) e (Xf,Yf)
    vector<vector<double>> Aresta;

    vector<vector<double>> Points;

    double Plane_Alpha;


    Stair();

    //Método referente à ordenacão dos degraus
    void SortSteps();

    //Método para obter as folhas presentes nos degraus
    void ExtractLeafsFromSteps();


    //Método obtendo os X, Y e Z mínimos e máximos da escada
    void CalculateStairProperties();


    void ModelStair(double Octree_Resolution);
    void ModelStair2D(double Octree_Resolution);


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

    MatrixXf OccupiedPoints;


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

    int Rho_Num;
    int Theta_Num;
    int Phi_Num;

    //Diferenca máxima entre o Rho do ponto e o Rho da célula para ser considerado como um voto
    double Max_Planes_Distance = 0.1;

    vector<vector<vector<int>>> Accumulator;

    //Variáveis para filtrar os planos a serem extraídos
    double Filter_Phi_Min;
    double Filter_Phi_Max;
    double Filter_Vote_Min;
    double Filter_Vote_Max;

    //Definindo as tolerâncias para o merge dos planos
    double delta_merge_Rho;

    double delta_Rho;
    double delta_Theta;
    double delta_Phi;

    //Definindo variáveis resultantes do histograma
    double Histogram_Dist_Min;
    double Histogram_Dist_Max;

    //Definindo características da escada
    int Min_Num_Steps;
    double Min_Step_Width;
    double Max_Step_Width;
    double Min_Step_Height;
    double Max_Step_Height;


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


    ///*** Lógica utilizando retas***
    //A identificacão da escada será sobre o vetor de folhas contidas dentro do Bounding Box usando uma lógica em 2D
    void StairDetection2D();

    //Agrupando as folhas que possuem o mesmo Z (estão no mesmo nível) em grupos
    vector<vector<OcTree::leaf_bbx_iterator>> GroupPlanesByZ(vector<OcTree::leaf_bbx_iterator> Leafs);

    //Obtendo a largura e comprimento do espaco definido pelas folhas
    vector<double> getParameter(vector<OcTree::leaf_bbx_iterator> Leafs);

    //Aplicando a Transformada de Hough para todos os níveis de Z, obtendo todas um grupo de grupo de retas (cada grupo de retas é referente à um nível de Z)
    vector<vector<Line*>> LineHoughTransform(double length, double width,vector<vector<OcTree::leaf_bbx_iterator>> Leafs);

    //Funcão que, para um determinado grupo de folhas, cria o acumulador, acumula os votos e extrai as retas desse grupo
    vector<vector<int>> AccumulatePoint2D(vector<OcTree::leaf_bbx_iterator> LeafZ);

    //Funcão que cria um grupo de objetos Line, referente às retas em um determinado nível de Z
    vector<Line*> createGroupLines(vector<vector<int>>Votes, double Z);

    //Agrupando as linhas por Rho e Theta (para identificar as retas recorrentes em vários níveis de Z)
    vector<vector<Line*>> GroupLineByRhoTheta(vector<vector<diane_octomap::Line*>> Lines);

    //Filtrando as retas que aparecem em muitos níveis de Z - provavelmente são paredes - ou que aparecem muitos poucos níveis de Z - provavelmente são ruídos)
    vector<vector<Line*>> FilterGroups(vector<vector<diane_octomap::Line*>> GroupLineByTheta, int Min_Size, int Max_Size);

    //Gerando um único objeto Line para cada grupo de Line's que possuem o mesmo Rho e Theta (armazenando o min_Z e o max_Z em que essa cada Line ocorre)
    vector<Line*> MergeGroupedLines(vector<vector<Line*>> GroupedLines);

    //Populando cada Line obtido com as folhas que possuam Z dentro dos limites e que estejam à uma distância mínima da reta
    void PopulateLines(vector<Line*>& Merged_Lines, vector<vector<OcTree::leaf_bbx_iterator>> Leaf_Groups);
    void PopulateLinesWithMatrix(vector<Line*>& Merged_Lines, vector<MatrixXf> Leaf_Groups_In_Matrix);


    //Funcao para verificar se existe uma folha dentro de um grupo de folhas
    bool GroupContainsLeaf(MatrixXf Leaf_Group, Vector3f Leaf);



    //Seperando as retas como segmento de reta
    vector<Line*> SegmentLines(vector<Line*> Lines);
    vector<Line*> SegmentLinesWithMatrix(vector<Line*> Lines);


    //Agrupando os segmentos de linhas que possuírem o mesmo Theta e que possuírem o intervalo aceitável
    vector<vector<Line*>> GroupLinesByThetaAndInterval(vector<diane_octomap::Line*> Segmented_Lines);


    //Realizando um Merge para todos os grupos de linhas com o mesmo Theta e que possuem o Rho's muito próximos
    vector<vector<Line*>> MergeSegmentedGroupsLines(vector<vector<Line*>> GroupThetaIntervalLines);

    vector<Line*> MergeSegmentedGroup(vector<Line*> SegmentedGroupLines);

    bool CanMergeLines(Line* LineA, Line* LineB);

    diane_octomap::Line* FitLine(Line* LineA, Line* LineB);

    vector<Line*> SortGroupLines(vector<Line*> GroupLines);


    //Buscando sequências nos grupos de linhas
    vector<vector<Line*>> SequenceFilter(vector<vector<Line*>> lines);

    bool VerifyLineSequence(vector<Line*> Group_Lines);

    void CompareStair(vector<Line*> list1, vector<Line*> list2);

    //Criando os objetos dos candidatos de escada
    vector<Stair*> CreateStairCandidates(vector<vector<Line*>> Sequenced_Groups);


    ///*** Fim da Lógica utilizando retas***


//    ///*** Lógica utilizando planos***

//    //A identificacão da escada será sobre o vetor de folhas contidas dentro do Bounding Box
//    void StairDetection();


//    vector<double> GetSpaceProperties();


//    //Metodos para Transformada de Hough
//    vector<vector<double>> PlanesHoughTransform(double length, double width, double height);


//    void InitializeAccumulator();


//    void AccumulatePoint(OcTree::leaf_bbx_iterator p);


//    vector<vector<double>> GetFilteredPlanes();



//    //Métodos para merge dos planos que são praticamente coplanares
//    vector<vector<double>> MergePlanes(vector<vector<double>> Planes);


//    bool CanMergePlanes(vector<double> PlaneA, vector<double> PlaneB);


//    vector<double> FitPlane(vector<double> PlaneA, vector<double> PlaneB);



//    //Método para agrupar os planos pelo (Theta e Phi)
//    vector<vector<vector<double>>> GroupPlanesByThetaPhi(vector<vector<double>> Planes);



//    //Métodos referentes à criacão do Histograma (Necessário? Está funcionando somente como um filtro)
//    vector<vector<vector<double>>> GenerateHistogram(vector<vector<vector<double>>> SameAnglesPlanes);


//    //Metodo para extracao dos planos que passaram pelo filtro do histograma
//    vector<vector<vector<double>>> ExtractValidPlanes(map<vector<vector<double>>, double> Planes_Dist_Map);


//    //Método de filtro do conjunto de planos
//    vector<vector<vector<double>>> FilterGroups(vector<vector<vector<double>>> Groups_Planes);


//    //Método que verifica se há uma sequência
//    bool VerifySequence(vector<vector<double>> Group_Planes);



//    vector<Stair*> StairCandidatesDetection(vector<vector<vector<double>>> Grouped_Planes);


//    void CleanStairsLeafs(vector<Stair*>& Detected_Stair_Candidates);


//    vector<Stair*> CleanStairSteps(vector<Stair*>& Detected_Stair_Candidates);


    vector<Stair*> ModelStairs(vector<Stair*>& Stair_Candidates);



    //Métodos referentes à aplicacão dos planos encontrados no octomap.
    Stair* ObtainStairCandidateFromGroup(vector<vector<double>> Group_Planes);


    //Métodos referentes à couts e à escrita em arquivos
    void PrintAccumulator();


    void WriteFilteredPlanesToFile(vector<vector<double>> Filtered_Planes);


    void WriteMergedPlanesToFile(vector<vector<double>> Merged_Planes);


    void WriteStairCandidateToFile(diane_octomap::Stair* Stair);


    void WriteModeledStairPropertiesToFile(diane_octomap::Stair* Stair);


    ///*** Lógica utilizando planos***


    virtual ~DianeOctomap();
};


}

#endif
