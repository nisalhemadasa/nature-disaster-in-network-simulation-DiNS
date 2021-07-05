#include "SensorRouting.h"
#include <iostream>
#include <iomanip>
#include <ctime>

namespace inet {
    Define_Module(SensorRouting);
    void SensorRouting::setSinkRouting(bool in){
        isSink=in;
    }
    void SensorRouting::setRiskIndex(double in){
        riskIndex=in;
        if(riskIndex<fireThreshold){
            Fire=true;
        }
    }
    void SensorRouting::initialize(){
        //----------address and log debug-----------------------
        runOnce = true;
        queuingTimeRecords = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/queuingTimeRecords.txt";
        std::ofstream myfile;
        myfile.open(queuingTimeRecords);
        myfile.close();
        //--------------------------------------

        parametersSize=par("parametersSize");
        updateTime=par("updateTime");
        maxNumberHops=par("maxNumberHops");
        initialBattery=par("initialBattery");
        minBattery=par("minBattery");
        thresholdBattery=par("thresholdBattery");
        thresholdRiskIndex=par("thresholdRiskIndex");
        minTrikleTime=par("minTrikleTime");
        maxTrikleTime=par("maxTrikleTime");
        maxNeighborTime=par("maxNeighborTime");
        maxDataSize=par("maxDataSize");
        meassurementSize=par("meassurementSize");
        headerLength=par("headerLength");
        maxpenalties=par("maxpenalties");
        fireThreshold=par("fireThreshold");
        trickleLimit=minTrikleTime;
        node=getParentModule();
        record=node->getSubmodule("record");
        wlan=node->getSubmodule("wlan", 0);
        mac=wlan->getSubmodule("mac");
        energy=node->getSubmodule("energyStorage");
        recordFuntions = check_and_cast<Record *>(record);
        macFuntions = check_and_cast<BOXMac *>(mac);
        energyFuntions = check_and_cast<power::IdealEpEnergyStorage *>(energy);
        myParameters.creationTime=simTime();
        myParameters.Battery=initialBattery;
        myParameters.MacName=macFuntions->getMacLayerAddress();
        myParameters.receptionPower=0;
        myParameters.queSize=macFuntions->getMacQueSize();
        myParameters.penalties=0;
        myParameters.parametersSize=parametersSize;
        riskIndex=7;
        myParameters.riskIndex=riskIndex; //Amgstrong index (>=4 no risk)
        myParameters.congestion=0;
        iniflag=true;
        Fire=false;
        myParameters.Fire=false;

        parentParameters=myParameters;
        if(isSink){
            myParameters.numberHops=0;
        }else{
            myParameters.numberHops=maxNumberHops;
        }
        trikleMsg = new cMessage("trikleEvent");
        trikleMsg->setKind(TRIKLEEVENT);
        updateMsg = new cMessage("updateEvent");
        updateMsg->setKind(UPDATEEVENT);
        if (isSink){
            trickleLimit=minTrikleTime;
        }else{
            trickleLimit=maxTrikleTime/2;
        }
        recordFuntions->setIsGateway(isSink);
        recordFuntions->recordLiveState(true); //record alive state
        scheduleAt(simTime()+updateTime*uniform(0,1),updateMsg);
        scheduleAt(simTime()+uniform(trickleLimit,2*trickleLimit),trikleMsg);
//        scheduleAt(simTime()+updateTime*1.5,updateMsg);
//        scheduleAt(simTime()+1.5*updateTime,trikleMsg);

        //-------------Q-learning related-----------------------------------------
        listenSuccessCount = 0;
        listenFailureCount = 0;
        transmitSuccessCount = 0;
        transmitFailureCount = 0;

        isDistributed = false;
        numberOfStates = 1;  //Number of destinations(gateways)
        numberOfActions = 1;    //transmit x numberOfNeighbors + receive (the initially only the receive would be there)
        gama = 0.8;
        epsilon = 1;
        prevMacQueueSize = 0;
        safestAngstromRiskIndex = 4.0;
        dangerAngstromRiskIndex = 1.0;
        packetReceptionDefaultAddress.setAddress("ff:ff:ff:ff:ff:fe");
        currentAction = packetReceptionDefaultAddress;
        currentState = 0;  //Currently there's only one gateway. So it's assigned the index 0
        queuingTime = 0.0;
        macFuntions->setQueuingTime(queuingTime);

        const char *iterationCount=par("iterationCount").stringValue();
        vector<int> iterationsCount = cStringTokenizer(iterationCount).asIntVector();

        if (iterationsCount[0] < 20)       //Evolution of epsilon; discounting factor
        {
            epsilon = 1;
        }
        else if (iterationsCount[0] < 40)
        {
            epsilon = 0.7;
        }
        else if (iterationsCount[0] < 60)
        {
            epsilon = 0.3;
        }
        else if (iterationsCount[0] < 130)
        {
            epsilon = 0.1;
        }
        else
        {
            epsilon = 0.0;
        }

        if (iterationsCount[0] < 75)   // Evolution of alpha; learning factor
        {
            alpha = 0.9;
        }
        else if (iterationsCount[0] < 90)
        {
            alpha = 0.1;
        }
        else if (iterationsCount[0] < 110)
        {
            alpha = 0.01;
        }
        else if(iterationsCount[0] < 130)
        {
            alpha = 0.001;
        }
        else
        {
            alpha = 0.0001;
        }

    }
    void SensorRouting::setOff(){
        cancelAndDelete(updateMsg);
        cancelAndDelete(trikleMsg);
        cDisplayString& dispStr = getContainingNode(this)->getDisplayString();
        dispStr.setTagArg("t", 0, "Dead");
        recordFuntions->recordLiveState(false); //record death state
    }
    void SensorRouting::addNeighbor(Packet *packet,Ptr<const BOXMacHeader> mac){
        sparameters_t neighbor;
        neighbor.MacName=mac->getSrcAddr(); //Not changed (It is correct as before)
        neighbor.receptionPower = packet->getTag<SignalPowerInd>()->getPower().get();
        auto parameters = packet->popAtFront<ParameterChunk>();
        neighbor.riskIndex=parameters->getRiskIndex();
        neighbor.numberHops=parameters->getNumberHops();
        neighbor.Battery=parameters->getBattery();
        neighbor.congestion=parameters->getCongestion();
        neighbor.Fire=parameters->getFire();
        if (neighbor.Fire == true){
            Fire=true;
            macFuntions->setFire();
        }
        neighbor.penalties=0;
        neighbor.creationTime=simTime();     //Reset time when I receive parameters
        neighbor.queSize=macFuntions->getMacQueSize();
        unsigned int i=0;
        while (i<neighborsVector.size()){
            if (neighborsVector[i].MacName==neighbor.MacName){
                neighborsVector[i]=neighbor;
                recordFuntions->recordParameters(neighbor, 2); // record update parameters
                break;
            }
            i++;
        }
        if (i==neighborsVector.size()){
            neighborsVector.push_back(neighbor);
            recordFuntions->recordParameters(neighbor, 1); // record add parameters
        }
        //--------------Q learning related----------------------------------------
        if(isDistributed && !isSink /*&& Fire*/){
            numberOfActions++;    //number of states increases as the number of neighbors increase (numberOfActions = number of neighbors)

           //        long long int MacValue;
           //        for (int i=0; i<neighborsVector[i].MacName.getAddressSize();i++){
           //            MacValue+=(long long int)neighborsVector[i].MacName.getAddressByte(i)*pow(256,i);
           //        }

           //        if(! Q.count(MacValue))     //adding one more state to the Q table and initialize all the actions to 0.0.
                   if(! Q.count(neighborsVector[i].MacName))     //adding one more state to the Q table and initialize all the actions to 0.0.
                   {
                       std::vector<float> qValues;

                       for(int i=0;i<= numberOfStates-1;i++)
                       {
                           qValues.push_back(0.0);
                       }

           //            Q.insert(std::pair<long long int,std::vector<float>>(MacValue,qValues));
                       Q.insert(std::pair<inet::MacAddress,std::vector<float>>(neighborsVector[i].MacName,qValues));
                       R.insert(std::pair<inet::MacAddress,std::vector<float>>(neighborsVector[i].MacName,qValues));
                   }
        }
        //----------------------------------------------------------------------------------------------------------------------------------
    }
    void SensorRouting::addNeighborData(Packet *packet, bool isReturn, int myMacAddress, int parentMacAddress){
        auto neighborData = packet->popAtFront<DataChunk>();
        int vectorSize=dataVector.size();
        unsigned int i=0;

        //-----------Q learning related-------------------------------------------------------------
        if(!isReturn){
            double timeStamp = atof(simTime().str().c_str());
            /* Once a new packet is received from a neighbor, its time stamp of collection and the number of measurements (eg: single data chunk has 2 measurements)
             * are inserted as a vector pair into the measurementQueuingTime map.
             * */
            if ( measurementQueuingTime.find(timeStamp) == measurementQueuingTime.end() ) {
                // if the key:timeStamp does not exist in the measurementQueuingTime map, insert it.
                // [map.find(k) searches the container(map) for an element with a key equivalent to k and returns an iterator to it if found, otherwise it returns an iterator to map::end
                measurementQueuingTime.insert(std::pair<double,int>(timeStamp, static_cast<int>(neighborData->getMacNameArraySize())));
            } else {
                // if the key exists in the map
                measurementQueuingTime[timeStamp] = measurementQueuingTime[timeStamp] + static_cast<int>(neighborData->getMacNameArraySize());
            }

            //=======================cumTimeToGatewayRecords log debug==================//
            if(!isSink && myMacAddress == 9){                                           //
               std::ofstream myfile;                                                    //
               std::string qTime = to_string(myMacAddress)
                       +"_"+ "in" + "_" + to_string(timeStamp) + "_" + to_string(measurementQueuingTime[timeStamp]);
               myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
               myfile << qTime + "\n";                                       //
               myfile.close();                                                          //
            }                                                                           //
            //========================cumTimeToGatewayRecords debug end=================//
        }
        //-------------------------------------------------------------------------------------------

        if (vectorSize>0){
            int lastDataSize=dataVector[dataVector.size()-1]->getMacNameArraySize()*meassurementSize;

            while(lastDataSize<=maxDataSize-meassurementSize && i<neighborData->getMacNameArraySize()){
                dataVector[dataVector.size()-1]->insertMacName(neighborData->getMacName(i));
                dataVector[dataVector.size()-1]->insertType(neighborData->getType(i));
                dataVector[dataVector.size()-1]->insertTime(neighborData->getTime(i));
                dataVector[dataVector.size()-1]->insertValue(neighborData->getValue(i));
                if (!isReturn){
                    recordFuntions->recordNewData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i)); //record new data
                    //---------------Q learning related----------------------------------
                    int packetTransmissionFailedFlag = 0;  // 1 if the packet has been failed during transmission and is returned
                    recordFuntions->recordEpisodicData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i),
                            parentMacAddress, myParameters.Battery, myParameters.riskIndex, myParameters.penalties,
                            myParameters.numberHops, queuingTime, myParameters.queSize, macFuntions->getCumulativeTimeToGateway(), packetTransmissionFailedFlag); // record data to create RL episodes
                    //--------------------------------------------------------------------
                }else{  // if the packet has been returned after a failed transmission
                    //---------------Q learning related----------------------------------
                    // when the transmission is failed, the parameters recorded are parameters of the parent (not my parameters), from whom the packet is returned.
                    int packetTransmissionFailedFlag = 1;  // 1 if the packet has been failed during transmission and is returned
                    double queuingTimeParent = -1.0;
                    double cumulativeTimeParent = -1.0;
                    recordFuntions->recordEpisodicData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i),
                            parentMacAddress, parentParameters.Battery, parentParameters.riskIndex, parentParameters.penalties,
                            parentParameters.numberHops, queuingTimeParent, parentParameters.queSize, cumulativeTimeParent, packetTransmissionFailedFlag); // record data to create RL episodes
                    //--------------------------------------------------------------------
                }
                i++;
                lastDataSize+=meassurementSize;
            }
        }
        if (i<neighborData->getMacNameArraySize()){
            auto remainingChunk = makeShared<DataChunk>();
            while(i<neighborData->getMacNameArraySize()){
                remainingChunk->insertMacName(neighborData->getMacName(i));
                remainingChunk->insertType(neighborData->getType(i));
                remainingChunk->insertTime(neighborData->getTime(i));
                remainingChunk->insertValue(neighborData->getValue(i));
                if (!isReturn){
                    recordFuntions->recordNewData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i)); //record new data
                    //---------------Q learning related----------------------------------
                    int packetTransmissionFailedFlag = 0;  // 1 if the packet has been failed during transmission and is returned
                    recordFuntions->recordEpisodicData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i),
                            parentMacAddress, myParameters.Battery, myParameters.riskIndex, myParameters.penalties,
                            myParameters.numberHops, queuingTime, myParameters.queSize, macFuntions->getCumulativeTimeToGateway(), packetTransmissionFailedFlag); // record data to create RL episodes
                    //--------------------------------------------------------------------
                }else{  // if the packet has been returned after a failed transmission
                    //---------------Q learning related----------------------------------
                    // when the transmission is failed, the parameters recorded are parameters of the parent (not my parameters), from whom the packet is returned.
                    int packetTransmissionFailedFlag = 1;  // 1 if the packet has been failed during transmission and is returned
                    double queuingTimeParent = -1.0;
                    double cumulativeTimeParent = -1.0;
                    recordFuntions->recordEpisodicData(neighborData->getTime(i), neighborData->getMacName(i), neighborData->getType(i), neighborData->getValue(i),
                            parentMacAddress, parentParameters.Battery, parentParameters.riskIndex, parentParameters.penalties,
                            parentParameters.numberHops, queuingTimeParent, parentParameters.queSize, cumulativeTimeParent, packetTransmissionFailedFlag); // record data to create RL episodes
                    //--------------------------------------------------------------------
                }
                i++;
            }
            dataVector.push_back(remainingChunk);
        }

        //=======================dataVector debug==================//
        if(!isSink && myMacAddress == 9){                                           //
           std::ofstream myfile;                                                    //
           myfile << "\n";                                                          //
           for(int i=0; i < dataVector.size(); i++){
               Ptr<DataChunk> chunk = dataVector.at(i);
           }
//           for (std::map<double,int>::iterator it=measurementQueuingTime.begin(); it!=measurementQueuingTime.end(); ++it){
//               std::string qTime = "queue_" + to_string(it->first) + "_" + to_string(it->second);
//               myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
//               myfile << qTime + "\n";
//               myfile.close();
//           }
           myfile << "\n";
           myfile.close();                                                          //
        }                                                                           //
        //========================dataVector end=================//
    }
    void SensorRouting::addMyData(double value, int type, double readTime){
        int vectorSize=dataVector.size();
        int i=0;

        //-----------Q learning related-------------------------------------------------------------
        double timeStamp = atof(simTime().str().c_str());
        /* Once a new measurement is collected, its time stamp of collection and the number of measurements (eg: single data chunk has 2 measurements)
         * are inserted as a vector pair into the measurementQueuingTime map.
         * */
        if ( measurementQueuingTime.find(timeStamp) == measurementQueuingTime.end() ) {
            // if the key:timeStamp does not exist in the measurementQueuingTime map, insert it.
            // [map.find(k) searches the container(map) for an element with a key equivalent to k and returns an iterator to it if found, otherwise it returns an iterator to
            // map::end
            measurementQueuingTime.insert(std::pair<double,int>(timeStamp, 1)); //only one measurement; temperature or humidity, is received to this function at once
        } else {
            // if the key exists
            measurementQueuingTime[timeStamp] = measurementQueuingTime[timeStamp] + 1;  //the next measurement, either the temperature or measurement is received at the same time
        }
        //=======================cumTimeToGatewayRecords log debug==================//
        if(!isSink && myMacAddress == 9){                                           //
           std::ofstream myfile;                                                    //
           std::string qTime = to_string(myMacAddress)
                   +"_"+ "in" + "_" + to_string(timeStamp) + "___" + to_string(measurementQueuingTime[timeStamp]);
           myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
           myfile << qTime + "\n";                                       //
           myfile.close();                                                          //
        }                                                                           //
        //========================cumTimeToGatewayRecords debug end=================//
        //-------------------------------------------------------------------------------------------

        if (vectorSize>0){
            int lastDataSize=dataVector[dataVector.size()-1]->getMacNameArraySize()*meassurementSize;
            if(lastDataSize<=maxDataSize-meassurementSize){
                dataVector[dataVector.size()-1]->insertMacName(myParameters.MacName);
                dataVector[dataVector.size()-1]->insertType(type);
                dataVector[dataVector.size()-1]->insertTime(readTime);
                dataVector[dataVector.size()-1]->insertValue(value);
                recordFuntions->recordNewData(readTime, myParameters.MacName, type, value); //record new data
                //---------------Q learning realated----------------------------------
                int parentMacAddress = 101; //parent Address is assigned to 101(non-existing node) if the chunk is measured and compiled in this node.
                int packetTransmissionFailedFlag = 0;  // 1 if the packet has been failed during transmissison and is returned
                recordFuntions->recordEpisodicData(readTime, myParameters.MacName, type, value, parentMacAddress, myParameters.Battery,
                        myParameters.riskIndex, myParameters.penalties, myParameters.numberHops, queuingTime, myParameters.queSize,
                        macFuntions->getCumulativeTimeToGateway(), packetTransmissionFailedFlag); // record data to create RL episodes
                //--------------------------------------------------------------------
                i=1;
            }
        }
        if (i==0){
            auto remainingChunk = makeShared<DataChunk>();
            remainingChunk->insertMacName(myParameters.MacName);
            remainingChunk->insertType(type);
            remainingChunk->insertTime(readTime);
            remainingChunk->insertValue(value);
            dataVector.push_back(remainingChunk);
            recordFuntions->recordNewData(readTime, myParameters.MacName, type, value); //record new data
            //---------------Q learning realated----------------------------------
            int parentMacAddress = 101; //parent Address is assigned to 101(non-existing node) if the chunk is measured and compiled in this node.
            int packetTransmissionFailedFlag = 0;  // 1 if the packet has been failed during transmissison and is returned
            recordFuntions->recordEpisodicData(readTime, myParameters.MacName, type, value, parentMacAddress, myParameters.Battery,
                    myParameters.riskIndex, myParameters.penalties, myParameters.numberHops, queuingTime, myParameters.queSize,
                    macFuntions->getCumulativeTimeToGateway(), packetTransmissionFailedFlag); // record data to create RL episodes
            //--------------------------------------------------------------------
        }
    }
    double SensorRouting::normalizeParameter(double parameter, vector<double> vranges){
        double jump=1/(double)vranges.size();
        int i=0;
        while(i<vranges.size()){
            if(parameter<vranges[i]){
                return jump*i;
            }
            i++;
        }
        return 1;
    }
    double SensorRouting::withFireTag(sparameters_t node){
          const char *classMargins=par("classMargins").stringValue();
          const char *batteryClassMargins=par("batteryClassMargins").stringValue();
          const char *congestionClassMargins=par("congestionClassMargins").stringValue();
          const char *memoryClassMargins=par("memoryClassMargins").stringValue();

        //Normalize risk index
        riskClasses = cStringTokenizer(classMargins).asDoubleVector();
        double nrisk=1-normalizeParameter(node.riskIndex,riskClasses);

        //Normalize battery
        vector<double> batteryClasses = cStringTokenizer(batteryClassMargins).asDoubleVector();
        double nbattery=1-normalizeParameter(node.Battery,batteryClasses);

        //Normalize number of hops
        vector<double> nhClasses{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};

        double numberhops = normalizeParameter(node.numberHops,nhClasses);
        //Normalize penalties. This reflects the level of congestion
        vector<double> penaltyClasses = cStringTokenizer(congestionClassMargins).asDoubleVector();
        double npenalties = normalizeParameter(node.penalties,penaltyClasses);

        //Normalize busy memory, in the units of number of slots occupied in the memory. Refer 'queueLength' in omnetpp.ini to get the maximum capacity of memory
        vector<double> memoryClasses = cStringTokenizer(memoryClassMargins).asDoubleVector();
        double nmemory = normalizeParameter(node.queSize,memoryClasses);

        //Calculate tag
        double tag;
        tag=floor(pow(26,5)*nrisk)+floor(pow(26,4)*numberhops)+floor(pow(26,3)*nbattery)+floor(pow(26,2)*npenalties)+floor(pow(26,1)*nmemory);
    }
    double SensorRouting::withoutFireTag(sparameters_t node){
        //Normalize battery
        vector<double> batteryClasses{2,3,5,7,10,14,20,28,40,58,82,118,168,240,343,490,700};
        double nbattery=1-normalizeParameter(node.Battery,batteryClasses);

        //Normalize number of hops
        vector<double> nhClasses{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
        double numberhops = normalizeParameter(node.numberHops,nhClasses);

        //Normalize penalties
        vector<double> penaltyClasses{5};
        while (penaltyClasses.back()<maxpenalties){
            penaltyClasses.push_back(penaltyClasses.back()+5);
        }
        double npenalties = normalizeParameter(node.penalties,penaltyClasses);

        //Calculate tag
        double tag;
        tag=floor(pow(25,3)*numberhops)+floor(pow(25,2)*npenalties);floor(pow(25,4)*nbattery);
    }
    double SensorRouting::getNodeTag(sparameters_t node){
        double tag;
        if (myParameters.Fire){
            tag = withFireTag(node);
        }else{
            tag = withoutFireTag(node);
        }
        return tag;
    }
    void SensorRouting::updateParentNode(){
        if(neighborsVector.size()>0 && !isSink){
            if(isDistributed /*&& myParameters.Fire*/)
            {
                if(currentAction == packetReceptionDefaultAddress)
                {
                    /* There's no significance importance of this line. What is intended is to assign some neighbor as
                     * my neighbor because the currentAction is 'listening'. And in such cases, we also avoid using
                     * the tags used in the centralized algorithm, because this part of the code runs under the distributed algorithm*/
                    parentParameters=neighborsVector[getRandomNeighbor(neighborsVector.size(), 0)];
                } else {
                    for(unsigned int i=1;i<neighborsVector.size();i++)
                    {
                        if(neighborsVector[i].MacName == currentAction)
                        {
                            parentParameters=neighborsVector[i];
                        }
                    }
                }
            } else {
                parentParameters=neighborsVector[0];
                double nodeTag=getNodeTag(parentParameters);
                for(unsigned int i=1;i<neighborsVector.size();i++){
                    if(nodeTag>getNodeTag(neighborsVector[i])){
                        nodeTag=getNodeTag(neighborsVector[i]);
                        parentParameters=neighborsVector[i];
                    }
                }
            }
        } else {
            parentParameters=myParameters;
        }
        recordFuntions->recordParameters(parentParameters, 4); // record parent parametes
    }
    double SensorRouting::getTrikleTime(){
        double time = uniform(trickleLimit, 2*trickleLimit);
//        double time = trickleLimit * 3/2;
        if (trickleLimit<maxTrikleTime/2){
            trickleLimit=2*trickleLimit;
        }
        return time;
    }
    void SensorRouting::updateMyparameters(){
        double battery;
        if (!isSink){
            battery=initialBattery+energyFuntions->getEnergyBalance().get();
            if (battery<=minBattery){
                macFuntions->setDead();
            }
        }else{
            battery=initialBattery;
        }
        double congestion=macFuntions->getCongestion();
        int numberHops=myParameters.numberHops;
        int queSize=macFuntions->getMacQueSize();
        if(!isSink){
            updateParentNode();
            if(parentParameters.MacName==myParameters.MacName){
                numberHops=maxNumberHops;
            }else{
                if (parentParameters.numberHops>=maxNumberHops){
                    numberHops=maxNumberHops;
                }else{
                    numberHops=parentParameters.numberHops+1;
                }
            }
        }
        bool changeFlag;
        //---------Nisal--------------------------
        if(!isDistributed and myParameters.Fire and !riskClasses.empty()){
            int myProximityToFire = getProximityToFire(myParameters.riskIndex);
            if(!(myProximityToFire == getProximityToFire(riskIndex))){  //checks if new riskiness value induce a change in the level of 'proximityToFire'
                changeActionDurationCentralizedAlgo(myProximityToFire);
            }
        }
        //----------------------------------------------
        changeFlag = abs(battery-myParameters.Battery)>=thresholdBattery;
        changeFlag = changeFlag || abs(riskIndex-myParameters.riskIndex)>=thresholdRiskIndex;
        changeFlag = changeFlag || numberHops != myParameters.numberHops;
        changeFlag = changeFlag || myParameters.Fire != Fire;
        if (changeFlag){
            if(!(numberHops==maxNumberHops && numberHops==myParameters.numberHops)){
                cancelEvent(trikleMsg);
                trickleLimit=minTrikleTime;
                scheduleAt(simTime()+getTrikleTime(),trikleMsg);
            }
            myParameters.Battery=battery;
            myParameters.riskIndex=riskIndex;
            myParameters.congestion=congestion;
            myParameters.numberHops=numberHops;
            myParameters.queSize=queSize;
            myParameters.Fire=Fire;
            recordFuntions->recordParameters(myParameters, 0); // record my parameters
            //----------------Q learning related--------------------------------------
            if(isDistributed /*&& Fire*/){
                 macFuntions->setMyParameterRiskiness(riskIndex);
            }
            //-----------------------------------------------------------------------
        }

        changeDisplayState(myParameters.numberHops);
        //changeDisplayState(myParameters.Fire);
    }
    int SensorRouting::getProximityToFire(double riskiness){
        /* The objective of this function is to create a mathematical relationship between the
         * Angstrom index of a node and its proximity to the fire front. Since there are 2 riskiness
         * boundaries, 2 weight coefficients are used to create the required skewed parabolic function.
         * The function has a higher value towards the boundary closer to the fire.
         * */
        double weight1 = 0.976;
        double weight2 = 0.332;
        double proximityToFireIndex;
        int proximityClass;

        // This is a parameter between 1 and 0 determining the distance of the node to the fire.
        if(riskiness > -3.8 and riskiness < -1.8){
            proximityToFireIndex = weight1 + weight2 * (1/pow((riskiness - 2.0),2.0));
        }else if(riskiness > 1.0 and riskiness < 3.0){
            proximityToFireIndex = weight1 * (1/pow((riskiness + 2.8),2.0)) + weight2;
        }else{
            proximityToFireIndex = weight1 * (1/pow((riskiness + 2.8),2.0)) + weight2 * (1/pow((riskiness - 2.0),2.0));
        }

        // The proximity is segmented to 3 classes using boundaries 0.4 and 0.2
        if(proximityToFireIndex >= 0.4){
            proximityClass = 2;
        }else if(proximityToFireIndex >= 0.2){
            proximityClass = 1;
        }else{
            proximityClass = 0;
        }
        return proximityClass;
    }
    bool SensorRouting::getSendFlag(){                                          //Demo solution
        return true;
    }
    void SensorRouting::sendSensorParameters(sparameters_t parameters){
        if(macFuntions->getMacQueSize()<macFuntions->getMaxMacQueSize()){
            auto macHeader = makeShared<BOXMacHeader>();
            macHeader->setSrcAddr(macFuntions->getMacLayerAddress());
            macHeader->setDestAddr(MacAddress::BROADCAST_ADDRESS);
            macHeader->setChunkLength(b(headerLength));
            macHeader->setType(BOXMAC_DATA);
            auto parametersChunk = makeShared<ParameterChunk>();
            parametersChunk->setBattery(parameters.Battery);
            parametersChunk->setCongestion(parameters.congestion);
            parametersChunk->setNumberHops(parameters.numberHops);
            parametersChunk->setQueSize(parameters.queSize);
            parametersChunk->setRiskIndex(parameters.riskIndex);
            parametersChunk->setFire(parameters.Fire);
            parametersChunk->setChunkLength(b(parameters.parametersSize));
            auto packet = new Packet("BOXMacData", parametersChunk);
            packet->insertAtFront(macHeader);
            packet->setKind(BOXMAC_DATA);
            packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::boxmac);
            macFuntions->sendToMAClayer(packet);
        }
    }
    void SensorRouting::sendSensorData(Ptr<DataChunk> mdata, MacAddress macName){
        int dataSize=mdata->getMacNameArraySize();
        mdata->setChunkLength(b(dataSize*meassurementSize));
        auto packet = new Packet("BOXMacData", mdata);
        auto macHeader = makeShared<BOXMacHeader>();
        macHeader->setSrcAddr(macFuntions->getMacLayerAddress());
        macHeader->setDestAddr(macName);
        macHeader->setChunkLength(b(headerLength));
        macHeader->setType(BOXMAC_DATA);
        packet->insertAtFront(macHeader);
        packet->setKind(BOXMAC_DATA);
        packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::boxmac);
        macFuntions->sendToMAClayer(packet);
    }
    void SensorRouting::removeOldNeighbors(){
        for(unsigned int i=0;i<neighborsVector.size();i++){
            if ((simTime().dbl()-neighborsVector[i].creationTime.dbl())>maxNeighborTime){//if a neighbor has been mute for a long time, them remove him.
                //---------------------Q learning realated--------------------------------------
                if(isDistributed && !isSink /*&& Fire*/){
                    removeQStates(i);
                }
                //---------------------------------------------------------------------------------
                recordFuntions->recordParameters(neighborsVector[i], 3); // record remove parametres
                char text[128];
                long long int MacValue;
                for (int i=0; i<neighborsVector[i].MacName.getAddressSize();i++){
                    MacValue+=(long long int)neighborsVector[i].MacName.getAddressByte(i)*pow(256,i);
                }
                sprintf(text,"removed neighbor: %d", MacValue);
                getSimulation()->getActiveEnvir()->alert(text);
                neighborsVector.erase(neighborsVector.begin() + i);
            }
        }
    }
    void SensorRouting::handleMessage(cMessage *msg){
        if (iniflag){
            myParameters.MacName=macFuntions->getMacLayerAddress();
            myParameters.queSize=macFuntions->getMacQueSize();
            myParameters.congestion=macFuntions->getCongestion();
            //-----------Q learning related----------------------------------------
            if(isDistributed /*&& Fire*/){
                macFuntions->setMyParameterRiskiness(myParameters.riskIndex);
                macFuntions->setIsDistributed(isDistributed);
            }
            /*initialize Q-table and Reward-table*/
            std::vector<float> qValues;

            for(int i=0;i<= numberOfStates-1;i++)
            {
                qValues.push_back(0.0);
            }

            Q.insert(std::pair<inet::MacAddress,std::vector<float>>(packetReceptionDefaultAddress,qValues));
            R.insert(std::pair<inet::MacAddress,std::vector<float>>(packetReceptionDefaultAddress,qValues));
            //---------------------------------------------------------------------
            recordFuntions->recordParameters(myParameters, 0); // record my parameters
            iniflag=false;
        }
        if (macFuntions->isDead()){
            setOff();
        }else{
            //====================================================================================================
            //-----------------------event count-----------------------------------------
            if(simTime()>1400 && !isSink){
                char text[128];
                sprintf(text," sensorNumber: %d \n listenSuccessCount: %d \n transmitSuccessCount: %d \n"
                        "listenFailureCount: %d \n transmitFailureCount: %d", macFuntions->getSensorNumber(), listenSuccessCount,
                        transmitSuccessCount, listenFailureCount, transmitFailureCount);
                getSimulation()->getActiveEnvir()->alert(text);
            }
            if(simTime()>1400 && isSink){
                char text[128];
                sprintf(text," SINK!!!.......\n sensorNumber: %d \n listenSuccessCount: %d \n transmitSuccessCount: %d \n"
                        "listenFailureCount: %d \n transmitFailureCount: %d", macFuntions->getSensorNumber(), listenSuccessCount,
                        transmitSuccessCount, listenFailureCount, transmitFailureCount);
                getSimulation()->getActiveEnvir()->alert(text);
            }
            //----------------------event count end------------------------------------------


            //========================================================================================================================

            if (msg->getKind()==BOXMAC_DATA){   //we have received a packet from a neighbor either explicitly or broadcasted
                auto packet = check_and_cast<Packet *>(msg);
                auto mac = packet->popAtFront<BOXMacHeader>();
                auto destinationAddress = mac->getDestAddr();
                if (destinationAddress.isBroadcast()){  //if broadcasted
                    addNeighbor(packet,mac);
                }else{
                    auto sourceAddress = mac->getSrcAddr(); //source address
                    //-----------------------address debug-------------------------------------------
                    if(runOnce && !isSink){
                        runOnce = false;
                        int msize=destinationAddress.getAddressSize();
                        for (int i=0; i<msize;i++){
                            myMacAddress+=(long long int)destinationAddress.getAddressByte(i)*pow(256,i);
                        }
                    }
                    int childMacAddress = 101;    //this initialized MAC address;'101' does not belong to any of the 100 nodes, or the gateway in the network
                    int msize=sourceAddress.getAddressSize();
                    for (int i=0; i<msize;i++){
                        childMacAddress+=(long long int)sourceAddress.getAddressByte(i)*pow(256,i);
                    }
                    childMacAddress = childMacAddress - 101;
                    //-----------------------address debug end-------------------------------------

                    addNeighborData(packet,false, myMacAddress, childMacAddress);  //if explicit

                    //----------------Q learning related-------------------------------
                    if(isDistributed /*&& Fire*/){
                        macFuntions->resetLastPacketRetransmissionTime();
                        macFuntions->resetRetransmissionCount();
                    }
                    //-----------------------------------------------------------------

                    for(unsigned int i=0;i<neighborsVector.size();i++){
                        if (neighborsVector[i].MacName==sourceAddress){
                            neighborsVector[i].penalties=0;
                            neighborsVector[i].creationTime=simTime();      //Reset time when I receive data
                            //---------------------Q learning related---------------------------------------------------------
                            if(isDistributed && !isSink /*&& Fire*/)
                            {
                                listenSuccessCount++;
                                currentAction = packetReceptionDefaultAddress;
                                updateQTable(updateQValue(2, neighborsVector[i].riskIndex));  //0-failure in packet transmission, 1-success, 2-packet reception
                                chooseAnAction();
                            }
                            //------------------------------------------------------------------------------------------------
                        }
                    }
                }
                delete(msg);
            }else if(msg->getKind()==BOXMAC_FAIL){  //we have not received the ack from the neighbor, after we've send it a packet
                auto packet = check_and_cast<Packet *>(msg);
                auto mac = packet->popAtFront<BOXMacHeader>();
                auto destinationAddress = mac->getDestAddr();
                //-----------------------address debug-------------------------------------------
                auto sourceAddress = mac->getSrcAddr(); //source address
                if(runOnce && !isSink){
                    runOnce = false;
                    int msize=sourceAddress.getAddressSize();
                    for (int i=0; i<msize;i++){
                        myMacAddress+=(long long int)sourceAddress.getAddressByte(i)*pow(256,i);
                    }
                }
                int parentMacAddress = 101;    //this initialized MAC address;'101' does not belong to any of the 100 nodes, or the gateway in the network.
                int msize=destinationAddress.getAddressSize();
                for (int i=0; i<msize;i++){
                    parentMacAddress+=(long long int)destinationAddress.getAddressByte(i)*pow(256,i);
                }
                parentMacAddress = parentMacAddress - 101;
                //-----------------------address debug end-------------------------------------
                for(unsigned int i=0;i<neighborsVector.size();i++){
                    if (neighborsVector[i].MacName==destinationAddress){
                        if (neighborsVector[i].penalties<maxpenalties){
                            neighborsVector[i].penalties++;
                            recordFuntions->recordParameters(neighborsVector[i], 2); // record update parameters
                        }else{
                            //---------------------Q learning related--------------------------------------
                            if(isDistributed && !isSink /*&& Fire*/){
                                removeQStates(i);
                            }
                            //---------------------------------------------------------------------------------
                            recordFuntions->recordParameters(neighborsVector[i], 3); // record remove parameters
                            neighborsVector.erase(neighborsVector.begin() + i);  // a new parent will be selected every time before forwarding a packet in 'UPDATEEVENT'
                        }
                        //---------------Q learning related-------------------------------------------------
                        if (isDistributed && !isSink /*&& Fire*/)
                        {
                            transmitFailureCount++;
                            updateQTable(updateQValue(0,-1000.0));  //0-transmission failure (no ack received), 1-transmission success 2-listening success 3-listening failure
                            chooseAnAction();
                        }
                        //------------------------------------------------------------------------------------
                        break;
                    }
                }
                addNeighborData(packet, true, myMacAddress, parentMacAddress);
            }else if (msg->getKind()==BOXMAC_SUCCESS){  //We have received the ack from the neighbor that we've send the last packet
                auto packet = check_and_cast<Packet *>(msg);
                auto mac = packet->popAtFront<BOXMacHeader>();
                auto destinationAddress = mac->getDestAddr();
                for(unsigned int i=0;i<neighborsVector.size();i++){
                    if (neighborsVector[i].MacName==destinationAddress){
                        neighborsVector[i].penalties=0;
                        neighborsVector[i].creationTime=simTime();   //Reset time when I receive ack
                        recordFuntions->recordParameters(neighborsVector[i], 2); // record update parameters
                        break;
                    }
                }
                //-----------------------address debug-------------------------------------------
                auto sourceAddress = mac->getSrcAddr(); //source address
                if(runOnce && !isSink){
                    runOnce = false;
                    int msize=sourceAddress.getAddressSize();
                    for (int i=0; i<msize;i++){
                        myMacAddress+=(long long int)sourceAddress.getAddressByte(i)*pow(256,i);
                    }
                }
                //-----------------------address debug end-------------------------------------
                //---------------Q learning related-------------------------------------------------
                if (isDistributed && !isSink && msg->hasPar("riskiness") /*&& Fire*/)
                {
                    transmitSuccessCount++;
                    queuingTimeCalculation();
                    updateQTable(updateQValue(1,packet->par("riskiness").doubleValue()));  //0-failure transmission, 1-transmission success 2-listening success 3-listening failure
                    chooseAnAction();
                }else if(!isDistributed && !isSink){
                    queuingTimeCalculation(); // added to record get the records of queuing times in sensorRLData files when 'isDistributed = false'. This could be removed when 'isDistributed = true'
                }
                //-----------------------------------------------------------------------------------

                delete(msg);
            }else if(msg->getKind()==UPDATEEVENT){  // forward packet to the parent node
                removeOldNeighbors();
                updateMyparameters();
                bool sendFlag=getSendFlag();
                if(sendFlag && !isSink && myParameters.MacName != parentParameters.MacName && myParameters.numberHops<maxNumberHops){
                    while(dataVector.size()>0 && !macFuntions->isDead()){
                        sendSensorData(dataVector[0],parentParameters.MacName);
                        dataVector.erase(dataVector.begin());
                    }
                }
//                removeOldNeighbors();
//                updateMyparameters();
                scheduleAt(simTime()+updateTime,updateMsg);
            }else if(msg->getKind()==TRIKLEEVENT){  // broadcast my-parameters to the neighborhood
                  sendSensorParameters(myParameters);
                scheduleAt(simTime()+getTrikleTime(),trikleMsg);
            //---------------Q learning related-------------------------------------------------
            }else if(msg->getKind()==BOXMAC_LISTENING_FAILED){//No data packets has been received after the checkInterval(listening time) has been timed out
                if (isDistributed && !isSink /*&& Fire*/){
                    listenFailureCount++;
                    currentAction = packetReceptionDefaultAddress; //Change this after finalizing the Q table structure
                    updateQTable(updateQValue(3,-1000.0));  //0-failure transmission, 1-transmission success 2-listening success 3-listening failure
                    chooseAnAction();
                }
            }
            //-----------------------------------------------------------------------------------
        }
    }
    void SensorRouting::changeDisplayState(long value){
        cDisplayString& dispStr = getContainingNode(this)->getDisplayString();
        if(value>=maxNumberHops){
            dispStr.setTagArg("t", 0, "unreachable");
        }else{
            dispStr.setTagArg("t", 0, value);
        }
    }

    void SensorRouting::changeActionDurationCentralizedAlgo(int myProximityToFire){

        /* Added by Nisal to the centralized algorithm.
         * depending on the proximity of the node to the fire, durations of the windows
         * of listening and sending changes
         * */
        const char *slotDurationsSet=par("slotDurationsSet").stringValue();
        const char *checkIntervalsSet=par("checkIntervalsSet").stringValue();
        // the index 0 of the following vectors are allocated to the most risky class
        vector<double> slotDurationsVector = cStringTokenizer(slotDurationsSet).asDoubleVector();
        vector<double> checkIntervalsVector = cStringTokenizer(checkIntervalsSet).asDoubleVector();

        if(!riskClasses.empty()){
            switch(myProximityToFire){
                case 0:{
                    macFuntions->setSlotDuration(slotDurationsVector[0]);    //Time spent in transmit state. Refer the slotDuration in .ini file
                    macFuntions->setCheckInterval(checkIntervalsVector[0]);   //Time spent in listen state.
                    break;
                }
                case 1:{
                    macFuntions->setSlotDuration(slotDurationsVector[1]);
                    macFuntions->setCheckInterval(checkIntervalsVector[1]);
                    break;
                }
                case 2:{
                    macFuntions->setSlotDuration(slotDurationsVector[2]);   //more closer to the fire than the other two cases
                    macFuntions->setCheckInterval(checkIntervalsVector[2]);
                    break;
                }
                default:{
                    double slotDuration  = par("slotDuration");
                    double checkInterval = par("checkInterval");
                    macFuntions->setSlotDuration(slotDuration);
                    macFuntions->setCheckInterval(checkInterval);
                    break;
                }
            }
        }
    }

    //-----------------Q-learning related functions--------------------------

    void SensorRouting::chooseAnAction() {

        double actionStrategy = (double)rand() / (RAND_MAX);
        int randomActionIndex = -100000;        //initializing with an unrealistic value
        inet::MacAddress bestActionMacAddress;
        bestActionMacAddress.setAddress("ff:ff:ff:ff:ff:ff");

        char text[128];

        // Selecting a action strategy; explore or exploit
        if (actionStrategy < SensorRouting::epsilon)
        {
            randomActionIndex = round(uniform(0,numberOfActions - 1));       //Randomly choose a possible action connected to the current state.
        }else{
            bestActionMacAddress = getBestActionMacAddress();//Take the action with the maximum value in the Q-table, corresponding to the current state
        }

        if(randomActionIndex == neighborsVector.size() || bestActionMacAddress == packetReceptionDefaultAddress){   //? what does "randomActionIndex == neighborsVector.size()" do?
            //prioritize listening in the FSM
            currentAction = packetReceptionDefaultAddress;
            macFuntions->setSlotDuration(0.0098 - 0.003);
            macFuntions->setCheckInterval(0.0004 + 0.0003);
        }else{
            //prioritize sending in the FSM
            currentAction = neighborsVector[randomActionIndex].MacName;
            macFuntions->setSlotDuration(0.0098 + 0.003);    //Time out of transmit
            macFuntions->setCheckInterval(0.0004 - 0.0003);   //Time out of listen
        }
    }


    void SensorRouting::updateQTable(float reward)
    {
        if (R.count(currentAction) && Q.count(currentAction)) {
            Q[currentAction][currentState] = reward;
        }
    }

    float SensorRouting::updateQValue(int feedback, double riskiness)
    {
        int nextState;
//        inet::MacAddress nextState;
        float updatedQValue;

        if(R.count(currentAction) && Q.count(currentAction)){
            R[currentAction][currentState] = calculateReward(feedback, riskiness);   //Calculate the rewards using acknowledgement data, and trickle time
            nextState = currentState;//Because we are considering  only one state at the moment
            updatedQValue = ((1 - SensorRouting::alpha) * Q[currentAction][currentState] + SensorRouting::alpha *
                    (R[currentAction][currentState] + (gama * getBestActionQValue()))); //static_cast<int>
        }
        return updatedQValue;
    }

    void SensorRouting::queuingTimeCalculation(){
        double lastPacketSendTime = macFuntions->getLastPacketSentTime();   //lastPacketSendTime = time stamp of the transmission of the packet, lastly dispatched from the MAC layer
        int lastPacketMeasurementCount = macFuntions->getLastPacketMeasurementCount();  //lastPacketMeasurementCount = number of data chunks (measurements) consisted in the packet lastly sent
        int i = 0;
        double totalQueuingTime = 0.0;

        //=======================cumTimeToGatewayRecords log debug==================//
        if(!isSink && myMacAddress == 9){                                           //
           std::ofstream myfile;                                                    //
           myfile << "\n";                                                          //
           for (std::map<double,int>::iterator it=measurementQueuingTime.begin(); it!=measurementQueuingTime.end(); ++it){
               std::string qTime = "queue_" + to_string(it->first) + "_" + to_string(it->second);
               myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
               myfile << qTime + "\n";
               myfile.close();
           }
           myfile << "\n";
           myfile.close();                                                          //
        }                                                                           //
        //========================cumTimeToGatewayRecords debug end=================//

        for (std::map<double,int>::iterator it=measurementQueuingTime.begin(); it!=measurementQueuingTime.end(); ++it){
            i += it->second;
            if(i <= lastPacketMeasurementCount){
                //=======================cumTimeToGatewayRecords log debug==================//
                if(!isSink && myMacAddress == 9){                                           //
                   std::ofstream myfile;                                                    //
                   std::string qTime = to_string(myMacAddress)
                           +"_"+ "out" + "_" + to_string(lastPacketSendTime) + "_" + to_string(lastPacketMeasurementCount);
                   myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
                   myfile << qTime + "\n";                                                  //
                   myfile.close();                                                          //
                }                                                                           //
                //========================cumTimeToGatewayRecords debug end=================//
                totalQueuingTime = totalQueuingTime + (lastPacketSendTime - it->first)*it->second;
                measurementQueuingTime.erase(it->first);
                if (i == lastPacketMeasurementCount) {
                    break;
                }
            }else{
                //=======================cumTimeToGatewayRecords log debug==================//
                if(!isSink && myMacAddress == 9){                                           //
                   std::ofstream myfile;                                                    //
                   std::string qTime = to_string(myMacAddress)
                           +"_"+ "out" + "_" + to_string(lastPacketSendTime) + "_" + to_string(lastPacketMeasurementCount);
                   myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
                   myfile << qTime + "\n";                                                  //
                   myfile.close();                                                          //
                }                                                                           //
                //========================cumTimeToGatewayRecords debug end=================//

                totalQueuingTime = totalQueuingTime + (lastPacketSendTime - it->first)*(it->second - (i - lastPacketMeasurementCount));
                measurementQueuingTime[it->first] = i - lastPacketMeasurementCount;
                break;
            }
        }

        float discountingFactor = 0.7; // 0 < discountingFactor < 1 , when discountingFactor gets higher, queuingTime maintains more up-to-date information. This is ideal for rapidly varying environments.
        queuingTime = queuingTime + discountingFactor*(totalQueuingTime/lastPacketMeasurementCount - queuingTime);
        macFuntions->setQueuingTime(queuingTime); //modifying the value of variable;'queuingTime' in BOXMac.cc
        //=======================cumTimeToGatewayRecords log debug==================//
        if(!isSink && myMacAddress == 9){                                           //
           std::ofstream myfile;                                                    //
           std::string qTime = to_string(myMacAddress)
                   +"_"+ "queuingTime" + "_" + to_string(queuingTime) ;             //
           myfile.open(queuingTimeRecords, std::ios_base::app); // append instead of overwrite
           myfile << qTime + "\n";                                                  //
           myfile.close();                                                          //
        }                                                                           //
        //========================cumTimeToGatewayRecords debug end=================//
    }


    float SensorRouting::calculateReward(int feedback, double neighborRiskiness)
    {
        float reward;
        //---------part A--------------------------------
        double A;
        double cumulativeTimeToGateway = macFuntions->getCumulativeTimeToGateway();
        double cumulativeBackoffTime = macFuntions->getCumulativeBackoffTime();
        macFuntions->resetCumulativeBackofftime();
        double ackReceptionTime = macFuntions->getAckReceptionTime();
        double lastPacketRetransmissionTime = macFuntions->getLastPacketRetransmissionTime();
        int retransmissionCount = macFuntions->getRetransmissionCount();
        double waitInterval = macFuntions->getWaitInterval();
        double delta;
        if(neighborRiskiness != -1000.0){
            if(safestAngstromRiskIndex > neighborRiskiness){
                if(neighborRiskiness > dangerAngstromRiskIndex){
                    delta = (neighborRiskiness - dangerAngstromRiskIndex)/(safestAngstromRiskIndex - dangerAngstromRiskIndex);
                    delta = delta * 1.3 - 0.3;  //the expression is written in two lines for simplification purposes
                }else{
                    delta = -0.3;
                }
            }else{
                delta = 1.0;
            }
        }

        macFuntions->resetAckReceptionTime();
        macFuntions->resetLastPacketRetransmissionTime();
        macFuntions->resetRetransmissionCount();

        //---------part B--------------------------------
        double B;
        double listeningStartTime = macFuntions->getListeningStartTime();
        double dataPacketReceptionTime = macFuntions->getDataPacketReceptionTime();
        double beta;
        if(safestAngstromRiskIndex > neighborRiskiness){
            if(neighborRiskiness > dangerAngstromRiskIndex){
                beta = (safestAngstromRiskIndex - neighborRiskiness)/(safestAngstromRiskIndex - dangerAngstromRiskIndex);
            }else{
                beta = 1.0;
            }
        }else{
            beta = 0.01;
        }

        macFuntions->resetListeningStartTime();
        macFuntions->resetDataPacketReceptionTime();

        //---------part C--------------------------------
        double C;
        int macQueueCapacity = macFuntions->getMaxMacQueSize();
        int currentMacQueueSize = macFuntions->getMacQueSize();
        double percentageMemoryOccupied = (double)currentMacQueueSize * 100/(double)macQueueCapacity;
        double theta = 1.0;
        if(percentageMemoryOccupied < 75.0){
            C = 0.0;
        }else{
            C = theta * (prevMacQueueSize - currentMacQueueSize)/macQueueCapacity;
        }

        //---------part D--------------------------------
        double D;
        float k = 0.0F;
        int sigma = 0.2;
        if(myParameters.riskIndex < 1.0)
        {
            k = 1.0F;
        }

        switch(feedback)
        {
        //###############################################################
        //########## write reward functions #############################
        //###############################################################
            case 0:     //transmission failure
                A = -1.0;
                B = 0.0;
                D = 0;
                break;
            case 1: //transmission success
                A = delta * (double)(1/(cumulativeBackoffTime + (retransmissionCount - 1)*waitInterval +
                        (ackReceptionTime - lastPacketRetransmissionTime) + queuingTime + cumulativeTimeToGateway));
                B = 0.0;
                D = sigma * k;
                break;
            case 2://reception success
                A = 0.0;
//                B = beta * ;
                B = 0;
                D = -sigma * k;
                break;
            case 3: //reception failure
                A = 0.0;
                B = -0.5;
                D = 0.0;
                break;
            default:
                reward = 0;
                break;
        }
        reward =  A + B + C + D;
        prevMacQueueSize = macQueueCapacity;

        return reward;
    }


    inet::MacAddress SensorRouting::getBestActionMacAddress()
    {//get the key of the map Q with the highest Q value. The action?
        float maxQvalue = -10000.0; // set an very high negative value as the initial approximation
//        long long int bestNextState;
        inet::MacAddress bestNextAction;

        for(std::map<inet::MacAddress,std::vector<float>>::iterator it = Q.begin(); it != Q.end(); it++)
        {
            for(int i=0; i <= it->second.size() - 1; i++)
            {
                if(maxQvalue < it->second[i])   // the next best action also could be selected from this loop
                {
                    maxQvalue = it->second[i];
                    bestNextAction = it->first;
                }
            }
        }

        return bestNextAction;
     }


    float SensorRouting::getBestActionQValue()
    {//get the key of the map Q with the highest Q value.i.e. The action.
        float maxQvalue = -10000.0; // set an very high negative value as the initial approximation
//        long long int bestNextState;
        inet::MacAddress bestNextAction;

        for(std::map<inet::MacAddress,std::vector<float>>::iterator it = Q.begin(); it != Q.end(); it++)
        {
            for(int i=0; i <= it->second.size() - 1; i++)
            {
                if(maxQvalue < it->second[i])   // the next best action also could be selected from this loop
                {
                    maxQvalue = it->second[i];
                    bestNextAction = it->first;
                }
            }
        }

        return maxQvalue;
    }

    int SensorRouting::getRandomNeighbor(int upperBound, int lowerBound)
    {
        int state;
        int range = (upperBound - lowerBound) + 1;

        //Get a random int value between 0 and (numberOfNeighbors - 1).
        state = lowerBound + int(range * rand() / (RAND_MAX + 1.0));

        return state;
    }

    void SensorRouting::removeQStates(int i)
    {
       numberOfActions--;   //number of Actions reduces and the number of neighbors reduces (numberOfActions = number of neighbors)

       if(Q.count(neighborsVector[i].MacName))
       {
           Q.erase(neighborsVector[i].MacName);
       }
    }
} //namespace
