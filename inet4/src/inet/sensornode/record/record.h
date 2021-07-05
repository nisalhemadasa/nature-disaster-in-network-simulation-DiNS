#ifndef __INET4_RECORD_H_
#define __INET4_RECORD_H_

#include <omnetpp.h>
#include "inet/sensornode/structures/structures.h"
#include <string>
#include <sys/stat.h>

#include <fstream>

using namespace omnetpp;
using namespace std;

namespace inet {

class Record : public cSimpleModule{
  protected:
    virtual ~Record();
    virtual void saveFile(string fAddress, vector<vector<double>>& inlist);
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    bool IsGateway;
    int MaxColumns;
  public:
    void recordParameters(sparameters_t parameters, int action); //0 mine, 1 add neighbor, 2 update neghbor, 3 remove neighbor, 4 Parent node
    void recordLiveState(int live); //0 alive, 1 death
    void recordNewData(double dataTime,MacAddress MacName,int rtype,double rvalue);
    void recordEpisodicData(double dataTime,MacAddress MacName,int rtype,double rvalue,
            int rParentAddress, double battery, double riskIndex, int penalties,
            int numberHops, double queuingTime, int queueSize, double cumTimeToGateway, int packetTransmissionFailedFlag);   //added by Nisal for storing episodic data for Q leaning
    void setIsGateway(bool sink);
    vector<vector<double>> rNodeParameters;
    vector<double> rTimeRecordParameters;
    vector<double> rAction; ////0 mine, 1 add neighbor, 2 update neghbor, 3 remove neighbor, 4 Parent node
    vector<double> rParametersCreationTime;
    vector<double> rParametersMacValue;
    vector<double> rFire;
    vector<double> rReceptionPower;
    vector<double> rBattery;
    vector<double> rCongestion;
    vector<double> rRiskIndex;
    vector<double> rNumberHops;
    vector<double> rQueueSize;
    vector<double> rPenalties;
    vector<vector<double>> rLiveState;
    vector<double> rTimeRecordLiveState;
    vector<double> rLive;
    vector<vector<double>> rSensorData;
    vector<double> rTimeRecordNewData;
    vector<double> rDataCreationTime;
    vector<double> rDataMacValue;
    vector<double> rType;
    vector<double> rValue;
    //by Nisal
    vector<vector<double>> rSensorRLData;
    vector<double> rTimeRecordNewDataRL;
    vector<double> rDataCreationTimeRL;
    vector<double> rDataMacValueRL;
    vector<double> rTypeRL;
    vector<double> rValueRL;
    vector<double> rParentAddressRL;
    vector<double> rBatteryRL;
    vector<double> rPenaltiesRL;
    vector<double> rRiskIndexRL;
    vector<double> rNumberHopsRL;
    vector<double> rQueuingTimeRL;
    vector<double> rQueueSizeRL;
    vector<double> rCumTimeToGateway;
    vector<double> rPacketTransmissionFailedFlag;
};

} //namespace

#endif
