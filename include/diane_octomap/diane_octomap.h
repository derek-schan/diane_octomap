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

#include <time.h>
#include <boost/thread.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>

//#include <eigen3/Eigen/Core>

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
    float min_X;
    float max_X;
    float min_Z;
    float max_Z;


    Line();

    //Método que atualiza os limites de X e Z do objeto Line
    void UpdateLimitsWithMatrix();

    //Ordenando a matriz de folhas por X
    void SortLeafMatrixByX();

    //Ordenando a matriz de folhas por Z
    void SortLeafMatrixByZ();


    //Método que executará Mínimos Quadrados para o conjunto de folhas de um Line, atualizando os parâmetros Rho e Theta das retas
    void UpdateLineParametersWithMinSquare();


    virtual ~Line();

};


//Estrutura que armazenará as folhas referentes à esse degrau e as informacões de um degrau (comprimento, largura e altura)
class Step
{
protected:

public:
    vector<OcTree::leaf_bbx_iterator> Leafs_In_Step;
    MatrixXf Leafs_Of_Step;

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


    void CalculateStepPropertiesWithMatrix();


    void SortLeafMatrixByZInStep();


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
    MatrixXf Leafs_Of_Stair;


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
    void ExtractLeafsFromStepsMatrix();

    bool StairContainsLeaf(Vector3f Leaf);



    //Método obtendo os X, Y e Z mínimos e máximos da escada
    void CalculateStairProperties();
    void CalculateStairPropertiesWithMatrix();


    //Método ordenando a matriz de folhas da escada por Z
    void SortLeafMatrixByZInStair();


    //Modelando as escadas
    void ModelStair(double Octree_Resolution);
    void ModelStair2D(double Octree_Resolution);
    void ModelStair2DWithMatrix(double Octree_Resolution);


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



    ///Variáveis utilizadas para publicar o passo a passo
    MatrixXf First_Filtered_Points;

    float x_min, x_max, y_min, y_max;

    vector<MatrixXf> HoughLinesPoints;

    vector<MatrixXf> FilteredHoughLinesPoints;

    vector<MatrixXf> SequencedLinesSegmentsPoints;



    ///Variáveis referentes à Transformada de Hough (2D e 3D)
    double Rho_Min;
    double Rho_Max;
    double Theta_Min;   //Em graus
    double Theta_Max;   //Em graus
//    double Phi_Min;     //Em graus (Somente utilizado no 3D)
//    double Phi_Max;     //Em graus (Somente utilizado no 3D)

    double Rho_Passo;   //Incremento em Rho
    double Theta_Passo; //Incremento em Theta
//    double Phi_Passo;   //Incremento em Phi (Somente utilizado no 3D)

    int Rho_Num;    //Número de intervalos no eixo de rho no espaco da transformada
    int Theta_Num;  //Número de intervalos no eixo de theta no espaco da transformada
//    int Phi_Num;  //Número de intervalos no eixo de phi no espaco da transformada (Somente utilizado no 3D)


//    //Diferenca máxima entre o Rho do ponto e o Rho da célula para ser considerado como um voto (Somente utilizado no 3D)
//    double Max_Planes_Distance = 0.1;


//    vector<vector<vector<int>>> Accumulator;

//    //Variáveis para filtrar os planos a serem extraídos (Somente utilizado no 3D)
//    double Filter_Phi_Min;
//    double Filter_Phi_Max;
//    double Filter_Vote_Min;
//    double Filter_Vote_Max;


    ///Definindo as tolerâncias para o merge dos planos
    double delta_merge_Rho;


//    ///Definindo as tolerâncias para o merge dos Planos detectados (Somente utilizado no 3D)
//    double delta_Rho;
//    double delta_Theta;
//    double delta_Phi;

//    ///Definindo variáveis resultantes do histograma de planos
//    double Histogram_Dist_Min;
//    double Histogram_Dist_Max;

    ///Definindo características padrões de uma escada
    int Min_Num_Steps;
    double Min_Step_Width;
    double Max_Step_Width;
    double Min_Step_Height;
    double Max_Step_Height;


    ///Variável que armazenará todas as escadas que foram modeladas
    vector<Stair*> Modeled_Stairs;


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


    //Método para obter e armazenar as folhas contidas dentro de uma Bounding Box (Deixar opcional o uso de uma Bounding Box - Atualmente, o filtro não deveria retirar nenhuma folha)
    void GetOccupiedLeafsOfBBX(OcTree* octree);


    ///*** Lógica utilizando retas***
    //A identificacão da escada será sobre o vetor de folhas contidas dentro do Bounding Box usando uma lógica em 2D
    void StairDetection2D();

    vector<MatrixXf> GroupPlanesByXY(MatrixXf Leafs);



    vector<MatrixXf> GroupPlanesByZ(vector<MatrixXf> Leafs);

    //Agrupando as folhas que possuem o mesmo Z (estão no mesmo nível) em grupos
    vector<MatrixXf> GroupPlanesByZ(MatrixXf Leafs);


    //Obtendo a largura e comprimento do espaco definido pelas folhas
    vector<double> GetSpaceParameters(MatrixXf Leafs);


    //Aplicando a Transformada de Hough para todos os níveis de Z, obtendo todas um grupo de grupo de retas (cada grupo de retas é referente à um nível de Z)
    vector<vector<Line*>> LineHoughTransform(double Space_Length, double Space_Width,vector<MatrixXf> Leafs);


    //Funcão que, para um determinado grupo de folhas, cria o acumulador, acumula os votos e extrai as retas desse grupo
    vector<vector<int>> Accumulation2D(MatrixXf LeafZ);


    //Funcão que cria um grupo de objetos Line, referente às retas em um determinado nível de Z
    vector<Line*> CreateGroupLines(vector<vector<int>>Votes, float Z);


    //Agrupando as linhas por Rho e Theta (para identificar as retas recorrentes em vários níveis de Z)
    vector<vector<Line*>> GroupLineByRhoTheta(vector<vector<diane_octomap::Line*>> Lines);


    //Filtrando as retas que aparecem em muitos níveis de Z - provavelmente são paredes - ou que aparecem muitos poucos níveis de Z - provavelmente são ruídos)
    vector<vector<Line*>> FilterGroups(vector<vector<diane_octomap::Line*>> GroupLineByTheta, int Min_Size, int Max_Size);


    //Gerando um único objeto Line para cada grupo de Line's que possuem o mesmo Rho e Theta (armazenando o min_Z e o max_Z em que essa cada Line ocorre)
    vector<Line*> MergeGroupedLines(vector<vector<Line*>> GroupedLines);

    //Populando cada Line obtido com as folhas que possuam Z dentro dos limites e que estejam à uma distância mínima da reta
    void PopulateLinesWithMatrix(vector<Line*>& Merged_Lines, vector<MatrixXf> Leaf_Groups_In_Matrix);


    //Funcao para verificar se existe uma folha dentro de um grupo de folhas
    bool GroupContainsLeaf(MatrixXf Leaf_Group, Vector3f Leaf);


    //Seperando as retas como segmento de reta
    vector<Line*> SegmentLinesWithMatrix(vector<Line*> Lines);


    //Agrupando os segmentos de linhas que possuírem o mesmo Theta e que possuírem o intervalo aceitável
    vector<vector<Line*>> GroupLinesByThetaAndInterval(vector<diane_octomap::Line*> Segmented_Lines);


    //Realizando um Merge para todos os grupos de linhas com o mesmo Theta e que possuem o Rho's muito próximos
    vector<vector<Line*>> MergeSegmentedGroupsLines(vector<vector<Line*>> GroupThetaIntervalLines);


    vector<Line*> MergeSegmentedGroup(vector<Line*> SegmentedGroupLines);


    bool CanMergeLines(Line* LineA, Line* LineB);


    Line* FitLineWithMatrix(Line* LineA, Line* LineB);


    //Ordenando o grupo de Lines por Rho
    vector<Line*> SortGroupLines(vector<Line*> GroupLines);


    //Buscando sequências nos grupos de linhas
    vector<vector<Line*>> SequenceFilter(vector<vector<Line*>> lines);


    bool VerifyLineSequence(vector<Line*> Group_Lines);



    //Criando os objetos dos candidatos de escada
    vector<Stair*> CreateStairCandidatesWithMatrix(vector<vector<Line*>> Sequenced_Groups);


    void UpdateStairProperties(vector<Stair*> StairCandidates);


    //Modelando as escadas
    vector<Stair*> ModelStairsWithMatrix(vector<Stair*>& Stair_Candidates);


    //Escrevendo um candidato à escada para plot
    void WriteStairCandidateToFileWithMatrix(diane_octomap::Stair* Stair);

    ///*** Fim da Lógica utilizando retas***




    ///*** Lógica utilizando planos*** (Não está completo)

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


//    vector<Stair*> ModelStairs(vector<Stair*>& Stair_Candidates);



//    //Métodos referentes à aplicacão dos planos encontrados no octomap.
//    Stair* ObtainStairCandidateFromGroup(vector<vector<double>> Group_Planes);


//    //Métodos referentes à couts e à escrita em arquivos
//    void PrintAccumulator();


//    //Método para escrita dos Planos filtrados
//    void WriteFilteredPlanesToFile(vector<vector<double>> Filtered_Planes);


//    //Métodos para escrita dos Planos Aglutinados
//    void WriteMergedPlanesToFile(vector<vector<double>> Merged_Planes);


//    //Escrevendo um candidato à escada para plot
//    void WriteStairCandidateToFile(diane_octomap::Stair* Stair);

    ///*** Lógica utilizando planos***







    //Escrevendo as informacões de uma escada modelada para plot no matlab
    void WriteModeledStairPropertiesToFile(diane_octomap::Stair* Stair);




    virtual ~DianeOctomap();
};


}

#endif
