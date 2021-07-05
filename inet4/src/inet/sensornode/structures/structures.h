#ifndef INET_LIBRARIES_STRUCTURES_H_
#define INET_LIBRARIES_STRUCTURES_H_
#include "inet/common/INETDefs.h"
#include "inet/common/ModuleAccess.h"
#include "inet/libraries/Eigen/Dense"

using namespace Eigen;
using namespace std;

typedef struct sparameters{ //Sensor node parameters
    omnetpp::simtime_t creationTime;    //Time when it was added or updated to neighbor list (if the diference of time is higher than a threshold, the sensor node is removed from the list)
    //Time when the node was to my neighbor list or the time the latest packet was received by it. (if the diference of time is higher than a threshold, the sensor node is removed from the list)
    inet::MacAddress MacName;   //Mac address of the sensor node
    bool Fire;
    double receptionPower;  //Power signal received from a SN in the neighbor discovery process
    double Battery; //Remaining energy in the SN
    double congestion;  //Free channel meassurement
    double riskIndex;   //Fire index based on a combination of the SN humidity and its temperature
    int numberHops; //Remaining hops to the base station based on its best parent
    int penalties;  //Number of failed comunications after the last successful comunication
    int queSize;    //Number of packages waiting in the send queue
    int parametersSize; //Size in bits of a neighbor discovery package
}sparameters_t;

#endif /* INET_LIBRARIES_STRUCTURES_H_ */
