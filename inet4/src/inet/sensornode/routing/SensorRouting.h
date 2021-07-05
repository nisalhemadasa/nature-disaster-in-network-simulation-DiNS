#ifndef __INET4_SENSORROUTING_H_
#define __INET4_SENSORROUTING_H_

#include <omnetpp.h>
#include "inet/linklayer/common/MacAddress_m.h"
#include "inet/power/storage/IdealEpEnergyStorage.h"
#include "inet/sensornode/linklayer/boxmac/BOXMac.h"
#include "inet/sensornode/record/record.h"
#include <vector>

#include "inet/sensornode/structures/DataChunk_m.h"
#include "inet/sensornode/structures/ParameterChunk_m.h"
#include "inet/physicallayer/common/packetlevel/SignalTag_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include <math.h>
#include <map>
#include <array>

using namespace omnetpp;
using namespace std;

#define TRIKLEEVENT         0
#define UPDATEEVENT         1

namespace inet {

    class SensorRouting : public cSimpleModule{
      protected:
        virtual void initialize();
        virtual void handleMessage(cMessage *msg);
        virtual double getTrikleTime();
        virtual void updateMyparameters();
        virtual void addNeighbor(Packet *packet,Ptr<const BOXMacHeader> mac);
        virtual void addNeighborData(Packet *packet, bool isReturn, int myMacAddress, int parentMacAddress);     //int myMacAddress and int parentMacAddress is added by Nisal
        virtual double getNodeTag(sparameters_t node);
        virtual void updateParentNode();
        virtual bool getSendFlag();
        virtual void sendSensorParameters(sparameters_t parameters);
        virtual void changeDisplayState(long value);
        virtual void setOff();
        virtual void sendSensorData(Ptr<DataChunk> mdata, MacAddress macName);
        virtual void removeOldNeighbors();
        virtual double normalizeParameter(double parameter, vector<double> vranges);
        virtual double withoutFireTag(sparameters_t node);
        virtual double withFireTag(sparameters_t node);

        //Added by Nisal
        void changeActionDurationCentralizedAlgo(int riskiness);   //changes the durations of listening and sending windows depending on the riskiness of the node
        int getProximityToFire(double riskiness);
        double getQueuingTime();    //send the current queuing time value to the instance where it's called.

        //-------------------------Q learning related----------------------------------------------------
        double initializeQlearning();
        void chooseAnAction();
        int getRandomAction(int upperBound, int lowerBound);
        int getRandomNeighbor(int upperBound, int lowerBound);
        void updateQTable(float reward);
        inet::MacAddress getBestActionMacAddress();
        float updateQValue(int feedback, double riskiness); //calculate the reward for the bellman equation
        float getBestActionQValue();
        float calculateReward(int feedback, double riskiness);        //get the reward of the previous actions from the environment
        inet::MacAddress getNextState();     //get the next state from the environment
        void removeQStates(int i);
        void queuingTimeCalculation();  //calculate the average queuing time of a measurement in the buffer of the node

        bool isDistributed;        //set this to 'true' when Q learning need to be run
        double safestAngstromRiskIndex;
        double dangerAngstromRiskIndex;
        double epsilon;
        int numberOfActions;  //0-transmit x numberOfNeighbors, 1-receive,
        double gama;
        double alpha;
        int numberOfStates;   //Number of destinations(gateways)
//        long long int currentState;  //set the current state. Initially, a random initial state is decided
        inet::MacAddress currentAction;
        inet::MacAddress packetReceptionDefaultAddress;
        int currentState;
//        std::map<long long int,std::vector<float>> R, Q;   //Reward and Q tables. vector<vector<float>> is equivalent to a 2d array
        std::map<inet::MacAddress,std::vector<float>> R, Q;   //Reward and Q tables. vector<vector<float>> is equivalent to a 2d array
        int prevMacQueueSize;
        double queuingTime;     //the average time a measurement stay buffered in a node
        int queueChunkCount;    //number of data-chunks in the
        double queueMovingAvgWindow[2] = {0.0, 0.0};

        int listenSuccessCount;
        int listenFailureCount;
        int transmitSuccessCount;
        int transmitFailureCount;
        std::map<double,int> measurementQueuingTime;    //map <measurement reception time(time the packet received in the node), number of measurements(no. of data chunks in the packet)>
        //--------------------------------------------------------------------------------------------------

        double initialBattery;
        double minBattery;  //Battery before SN turn off
        double thresholdBattery;    //Minimum value to reset trickle timer
        double thresholdRiskIndex;  //Minimum value to reset trickle timer

        double minTrikleTime;   //Minimum interval for sending neighbor discovery package
        double maxTrikleTime;   //Maximum interval for sending neighbor discovery package
        double maxNeighborTime; //Threshold to delete SN in neidhbor list

        bool isSink;    //Flag for base station
        int maxNumberHops;  //Value to avoid infinite loops
        int maxDataSize;    //Maximum package size in bits
        int meassurementSize;   //Size packages with plant states in bits
        int headerLength;   //Size of mac header in bits
        double trickleLimit;    //trickle time is uniform(trickleLimit,2*trickleLimit)
        double updateTime;  //Time for checking own parameters
        int parametersSize; //Neighbor discovery package size in bits

        int maxpenalties; //Parameter to erase neighbor
        double fireThreshold; //Triger fire alert
        string iterationCount;
        bool Fire;
        sparameters_t myParameters; //Struct with my parameters
        sparameters_t parentParameters; //Struct with the parameters of the best candidate for parent node
        cModule *node;
        cModule *wlan;
        cModule *mac;
        cModule *sensing;
        cModule *energy;
        cModule *record;
        Record *recordFuntions;
        BOXMac *macFuntions;
        power::IdealEpEnergyStorage *energyFuntions;
        //SensorUnit *sensingFuntions;
        vector<sparameters_t> neighborsVector;
        cMessage *trikleMsg;
        cMessage *updateMsg;
        vector<Ptr<DataChunk>> dataVector;

        bool iniflag;

        double riskIndex;

        //Added by Nisal-------------------------------------
        vector<double> riskClasses;
        vector<double> riskFunctionWeights;
        //-------------debug--------------------
        bool runOnce;
        int myMacAddress;
        std::string queuingTimeRecords;
        //---------------------------------------

      public:
        void addMyData(double value, int type, double readTime);
        void setRiskIndex(double in);
        void setSinkRouting(bool in);

    };

} //namespace

#endif
