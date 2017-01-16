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


    virtual ~DianeOctomap();
};


}

#endif
