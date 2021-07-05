#include "record.h"

namespace inet {

Define_Module(Record);

void Record::saveFile(string fAddress, vector<vector<double>>& inlist){
    ofstream myfile;
    long int rsize=inlist[0].size();     //Number of columns
    long int csize=inlist.size();        //Number of rows
    long int nf;                         //File number
    for (long int h=0;h<rsize;h+=MaxColumns){
        nf=h/MaxColumns;
        //fAddress=fAddress+to_string(nf)+".csv";
        myfile.open(fAddress+to_string(nf)+".csv");
        for (long int i=0; i<csize; i++){
            for (long int j=h; (j<h+MaxColumns && j<rsize); j++){
                myfile << inlist[i][j] << " ";
            }
            myfile << '\n';
        }
        myfile.close();
    }
}

void Record::setIsGateway(bool sink){
    IsGateway=sink;
}
Record::~Record(){
    cModule *node;
    node=getParentModule();
    int index;
    index=node->getIndex();
    const char *RA;
    RA = par("RecAddress").stringValue();
    string DataAddress;
    string ParametersAddress;
    string LiveAddress;
    string RLDataAddress;
    string RecAddress(RA);
    string sindex = to_string(index);
    string NodeAddress;
    if(IsGateway){
        NodeAddress = RecAddress + "/gateway" + sindex;
    }else{
        NodeAddress = RecAddress + "/node" + sindex;
    }
    DataAddress = NodeAddress + "/sensorData";
    ParametersAddress = NodeAddress + "/nodeParameters";
    LiveAddress = NodeAddress + "/liveState";
    RLDataAddress = NodeAddress + "/sensorRLData";
    int dir_err;
    dir_err = mkdir(NodeAddress.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //liveState
    rLiveState.push_back(rTimeRecordLiveState);
    rLiveState.push_back(rLive);
    saveFile(LiveAddress,rLiveState);
    //nodeParameters
    rNodeParameters.push_back(rTimeRecordParameters); //1
    rNodeParameters.push_back(rAction); //2
    rNodeParameters.push_back(rParametersCreationTime); //3
    rNodeParameters.push_back(rParametersMacValue); //4
    rNodeParameters.push_back(rFire); //5
    rNodeParameters.push_back(rReceptionPower); //6
    rNodeParameters.push_back(rBattery); //7
    rNodeParameters.push_back(rCongestion); //8
    rNodeParameters.push_back(rRiskIndex); //9
    rNodeParameters.push_back(rNumberHops); //10
    rNodeParameters.push_back(rQueueSize); //11
    rNodeParameters.push_back(rPenalties); //12
    saveFile(ParametersAddress,rNodeParameters);
    //sensorData
    rSensorData.push_back(rTimeRecordNewData); //1
    rSensorData.push_back(rDataCreationTime); //2
    rSensorData.push_back(rDataMacValue); //3
    rSensorData.push_back(rType); //4
    rSensorData.push_back(rValue); //5
    saveFile(DataAddress,rSensorData);
    //sensorRLData (by Nisal)
    rSensorRLData.push_back(rTimeRecordNewDataRL); //1
    rSensorRLData.push_back(rDataCreationTimeRL); //2
    rSensorRLData.push_back(rDataMacValueRL); //3
    rSensorRLData.push_back(rTypeRL); //4
    rSensorRLData.push_back(rValueRL); //5
    rSensorRLData.push_back(rParentAddressRL); //6
    rSensorRLData.push_back(rBatteryRL); //7
    rSensorRLData.push_back(rRiskIndexRL); //8
    rSensorRLData.push_back(rPenaltiesRL); //9
    rSensorRLData.push_back(rNumberHopsRL); //10
    rSensorRLData.push_back(rQueuingTimeRL); //11
    rSensorRLData.push_back(rQueueSizeRL); //12
    rSensorRLData.push_back(rCumTimeToGateway); //13
    rSensorRLData.push_back(rPacketTransmissionFailedFlag); //14
    saveFile(RLDataAddress,rSensorRLData);

}
void Record::recordParameters(sparameters_t parameters, int action){
    rTimeRecordParameters.push_back(simTime().dbl());
    rAction.push_back(action);
    rParametersCreationTime.push_back(parameters.creationTime.dbl());

    long long int MacValue=0;
    int msize=parameters.MacName.getAddressSize();
    for (int i=0; i<parameters.MacName.getAddressSize();i++){
        MacValue+=(long long int)parameters.MacName.getAddressByte(i)*pow(256,i);
    }
    //MacValue+=parameters.MacName.getAddressByte(i);
    rParametersMacValue.push_back(MacValue);
    rFire.push_back(parameters.Fire);
    rReceptionPower.push_back(parameters.receptionPower);
    rBattery.push_back(parameters.Battery);
    rCongestion.push_back(parameters.congestion);
    rRiskIndex.push_back(parameters.riskIndex);
    rNumberHops.push_back(parameters.numberHops);
    rQueueSize.push_back(parameters.queSize);
    rPenalties.push_back(parameters.penalties);
    //0 mine, 1 add neighbor, 2 update neghbor, 3 remove neighbor, 4 Parent node
}
void Record::recordLiveState(int live){
    rTimeRecordLiveState.push_back(simTime().dbl());
    rLive.push_back(live);
    //1 alive, 0 death

}
void Record::recordNewData(double dataTime,MacAddress MacName,int rtype,double rvalue){
    rTimeRecordNewData.push_back(simTime().dbl());
    rDataCreationTime.push_back(dataTime);
    int MacValue=0;
    int msize=MacName.getAddressSize();
    for (int i=0; i<msize;i++){
        MacValue+=(long long int)MacName.getAddressByte(i)*pow(256,i);
        //MacValue+=MacName.getAddressByte(i);
    }

    rDataMacValue.push_back(MacValue);
    rType.push_back(rtype);
    rValue.push_back(rvalue);
}

//-------Added by Nisal for Q learning. Records the data needed to create the episodes for training.-----
void Record::recordEpisodicData(double dataTime,MacAddress MacName,int rtype,double rvalue,
        int parentAddress, double battery, double riskIndex, int penalties,
        int numberHops, double queuingTime, int queueSize, double cumTimeToGateway, int packetTransmissionFailedFlag){
    rTimeRecordNewDataRL.push_back(simTime().dbl());
    rDataCreationTimeRL.push_back(dataTime);
    int MacValue=0;
    int msize=MacName.getAddressSize();
    for (int i=0; i<msize;i++){
        MacValue+=(long long int)MacName.getAddressByte(i)*pow(256,i);
        //MacValue+=MacName.getAddressByte(i);
    }

    rDataMacValueRL.push_back(MacValue);
    rTypeRL.push_back(rtype);
    rValueRL.push_back(rvalue);
    rParentAddressRL.push_back(parentAddress);
    rBatteryRL.push_back(battery);
    rRiskIndexRL.push_back(riskIndex);
    rPenaltiesRL.push_back(penalties);
    rNumberHopsRL.push_back(numberHops);
    rQueuingTimeRL.push_back(queuingTime);
    rQueueSizeRL.push_back(queueSize);
    rCumTimeToGateway.push_back(cumTimeToGateway);
    rPacketTransmissionFailedFlag.push_back(packetTransmissionFailedFlag);
}
//----------------------------------------------------------------------------------------------------------

void Record::initialize(){
    MaxColumns=par("MaxColumns");
}

void Record::handleMessage(cMessage *msg){
}

} //namespace
