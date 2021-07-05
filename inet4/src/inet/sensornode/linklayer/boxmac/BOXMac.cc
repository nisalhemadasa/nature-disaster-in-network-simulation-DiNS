#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/queue/IPassiveQueue.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "BOXMac.h"
#include "BOXMacHeader_m.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include <fstream>


namespace inet {

using namespace physicallayer;
    Define_Module(BOXMac);
    //Access external modules
    void BOXMac::setSlotDuration(double value){
        slotDuration=value;
    }
    void BOXMac::setCheckInterval(double value){
        checkInterval=value;
    }
    //---debug---------------
    double  BOXMac::getSlotDuration(){
        return slotDuration;
    }
    double  BOXMac::getCheckInterval(){
        return checkInterval;
    }
    //-----------------------
    void BOXMac::setWaitInterval(double value){
        waitInterval=value;
    }
    double BOXMac::getWaitInterval(){
        return waitInterval;
    }
    void BOXMac::setSendDelay(double value){
        sendDelay=value;
    }
    void BOXMac::setSleepDuration(double value){
        sleepDuration=value;
    }
    double BOXMac::getCongestion(){
        return congestion;
    }
    int BOXMac::getMacQueSize(){
        return macQueue.size();
    }
    int BOXMac::getMaxMacQueSize(){
        return queueLength;
    }
    MacAddress BOXMac::getMacLayerAddress(){
        return address;
    }
    bool BOXMac::isDead(){
        return killFlag;
    }
    double BOXMac::getAckReceptionTime(){
        return ackReceptionTime;
    }
    void BOXMac::resetAckReceptionTime(){
        ackReceptionTime = 0.0;
    }
    double BOXMac::getLastPacketRetransmissionTime(){
        return lastPacketRetransmissionTime;
    };
    void BOXMac::resetLastPacketRetransmissionTime(){
        lastPacketRetransmissionTime = 0.0;
    }
    int BOXMac::getRetransmissionCount(){
        return retransmissionCount;
    }
    void BOXMac::resetRetransmissionCount(){
        retransmissionCount = 0;
    }
    double BOXMac::getListeningStartTime(){
        return listeningStartTime;
    }
    void BOXMac::resetListeningStartTime(){
        listeningStartTime = 0.0;
    }
    double BOXMac::getDataPacketReceptionTime(){
        return dataPacketReceptionTime;
    }
    void BOXMac::resetDataPacketReceptionTime(){
        dataPacketReceptionTime = 0.0;
    }
    void BOXMac::setMyParameterRiskiness(double value){
        myParameterRiskiness = value;
    }
    void BOXMac::setIsDistributed(bool value){
        isDistributed = value;
    }
    double BOXMac::getCumulativeBackoffTime(){
        return cumulativeBackoffTime;
    }
    void BOXMac::resetCumulativeBackofftime(){
        cumulativeBackoffTime = 0.0;
    }
    double BOXMac::getCumulativeTimeToGateway(){
        return cumulativeTimeToGateway;
    }
    void BOXMac::setQueuingTime(double value){
        queuingTime = value;
    }
    double BOXMac::getTransmissionDelay(){
        return transmissionDelay;
    }
    double BOXMac::getLastPacketSentTime(){
        return lastPacketSendTime;
    }
    int BOXMac::getLastPacketMeasurementCount(){
        return lastMeasurementSendCount;
    }
    //------------------debug--------------------------
    int BOXMac::getSensorNumber(){
        return sensorNumber;
    }
    //------------debug end-----------------------------
    void BOXMac::setFire(){
        fire = true;
    }
    void BOXMac::setOff(){
        cancelEvent(wakeup);
        cancelEvent(stop_preambles);
        cancelEvent(ack_tx_over);
        cancelEvent(start_boxmac);
        cancelEvent(switch_preamble_phase);
        cancelEvent(delay_for_ack_within_remote_rx);
        cancelEvent(switching_done);
        cancelEvent(periodicWakeup);
        offFlag = true;
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        macState = SLEEP;
    }
    void BOXMac::setOn(){
        if (offFlag && !killFlag){
            setSleep();
            offFlag = false;
        }
    }
    void BOXMac::setDead(){
        if (!isSink){
            killFlag = true;
            cDisplayString& dispStr = getContainingNode(this)->getDisplayString();
            dispStr.setTagArg("t", 0, "Dead");
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
            macState = SLEEP;
        }
    }
    void BOXMac::sendToMAClayer(Packet * packet){
          //-----------------------address debug-------------------------------------------
            int queueSize = macQueue.size();
            int MacValueMy=0;
            int msize=address.getAddressSize();
            for (int i=0; i<msize;i++){
                MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
            }
            if(MacValueMy == 9){
                int g = 0;
            }
          //-----------------address debug end-------------------------------------------
        macQueue.push_back(packet);
    }

    void BOXMac::setSleep(){
        cancelEvent(listenChannel_timeout);
        changeCongestion(-1); //Reset congestion
        double backofftime=uniform(0,congestion);
//        double backofftime=congestion/2;

        if (isSink){

            //=======================model sim=====================================
            //-----------------------address debug---------------------------------
            int MacValueMy=0;
            int msize=address.getAddressSize();
            for (int i=0; i<msize;i++){
                MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
            }
            //---------------------------------------------------------------------

            std::string radioState = "NODE_RADIO_STATE";
            std::string radioStateValue = to_string(MacValueMy) + "_" + "CCA" +\
                    "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));
            std::ofstream myfile;
            myfile.open (modelSimRadioStateFileLocation);
            myfile << radioStateValue;
            myfile.close();

            if(simTime() > 5.0 && simTime() < 1400.0){
                myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                myfile << radioStateValue + "\n";
                myfile.close();
            }
            //======================================================================
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            macState = CCA_ON;
            scheduleAt(simTime()+backofftime+checkInterval, listenChannel_timeout);
        }else if(isCCASleep){
           //=======================model sim=====================================
           //-----------------------address debug---------------------------------
           int MacValueMy=0;
           int msize=address.getAddressSize();
           for (int i=0; i<msize;i++){
               MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
           }
           //---------------------------------------------------------------------

           std::string radioState = "NODE_RADIO_STATE";
           std::string radioStateValue = to_string(MacValueMy) + "_" + "SLEEP" +\
                   "_" + "sl" + "_" + to_string(atof(simTime().str().c_str()));
           std::ofstream myfile;
           myfile.open (modelSimRadioStateFileLocation);
           myfile << radioStateValue;
           myfile.close();

           if(simTime() > 5.0 && simTime() < 1400.0){
               myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
               myfile << radioStateValue + "\n";
               myfile.close();
           }
           //======================================================================

//            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
            macState = SLEEP;
            if(isDistributed /*&& fire*/){
                cumulativeBackoffTime += backofftime;
            }
            scheduleAt(simTime() + backofftime, wakeup);
        }else{
           //=================model sim=========================================
            //-----------------------address debug---------------------------------
               int MacValueMy=0;
               int msize=address.getAddressSize();
               for (int i=0; i<msize;i++){
                   MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
               }
               //-----------------------------------------------------------------
               std::string radioState = "NODE_RADIO_STATE";
               std::string radioStateValue = to_string(MacValueMy) + "_" + "SLEEP" +\
                       "_" + "sl" + "_" + to_string(atof(simTime().str().c_str()));
               std::ofstream myfile;
               myfile.open (modelSimRadioStateFileLocation);
               myfile << radioStateValue;
               myfile.close();

               if(simTime() > 5.0 && simTime() < 1400.0){
                   myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                   myfile << radioStateValue + "\n";
                   myfile.close();
               }
           //====================================================================

            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
            macState = SLEEP;
            if(!periodicWakeupInitiated)
            {
                scheduleAt(simTime() + sleepDuration+backofftime, wakeup);
                wakeUpTime = atof(simTime().str().c_str()) + sleepDuration+backofftime;
            }
            cancelEvent(periodicWakeup);
            cancelEvent(start_boxmac);
            scheduleAt(simTime() + checkInterval, periodicWakeup);
        }
    }
    void BOXMac::setBackOff(){
        cancelEvent(listenChannel_timeout);
        changeCongestion(1); //Increase congestion
        double backofftime=uniform(0,congestion);
//        double backofftime=congestion/2;

        if(packetReceivedDuringCCA){
            CCAAttempt1Clear = false;
            CCAAttempt1ClearCount = 0;
            macState = SEND_ACK;
//            scheduleAt(simTime()+0.000001, delay_for_ack_within_remote_rx);

        }else if(!isSink){
            //=================model sim=========================================
           //-----------------------address debug-------------------------------
           int MacValueMy=0;
           int msize=address.getAddressSize();
           for (int i=0; i<msize;i++){
               MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
           }
           //-------------------------------------------------------------------

           std::string radioState = "NODE_RADIO_STATE";
           std::string radioStateValue = to_string(MacValueMy) + "_" + "SLEEP" +\
                   "_" + "sl" + "_" + to_string(atof(simTime().str().c_str()));
           std::ofstream myfile;
           myfile.open (modelSimRadioStateFileLocation);
           myfile << radioStateValue;
           myfile.close();

           if(simTime() > 5.0 && simTime() < 1400.0){
               myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
               myfile << radioStateValue + "\n";
               myfile.close();
           }
           //====================================================================

            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
            macState = SLEEP;
            scheduleAt(simTime()+backofftime, wakeup);
        }else{
           //=================model sim=========================================
           //-----------------------address debug-------------------------------
           int MacValueMy=0;
           int msize=address.getAddressSize();
           for (int i=0; i<msize;i++){
               MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
           }
           //-------------------------------------------------------------------

           std::string radioState = "NODE_RADIO_STATE";
           std::string radioStateValue = to_string(MacValueMy) + "_" + "CCA" +\
                   "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));
           std::ofstream myfile;
           myfile.open (modelSimRadioStateFileLocation);
           myfile << radioStateValue;
           myfile.close();

           if(simTime() > 5.0 && simTime() < 1400.0){
               myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
               myfile << radioStateValue + "\n";
               myfile.close();
           }
           //====================================================================
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            macState = CCA_ON;
            scheduleAt(simTime()+backofftime+checkInterval, listenChannel_timeout);
        }
    }
    void BOXMac::changeCongestion(int direction){
        if (direction<0){
            congestion=minimumBackOffTime;  //Reset congestion
        }else{
            if (congestion<maximumBackOffTime/2){
                congestion=congestion*2;    //Increase congestion
            }else{
                congestion=maximumBackOffTime;
            }
        }
    }
    void BOXMac::returnFailedPackage(){
        auto packet = macQueue.front()->dup();
        packet->setKind(BOXMAC_FAIL);
        cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");
        sendDirect(packet, sensorRouting, "macPackageIn");
    }
    void BOXMac::returnSuccessPackage(Packet *pkt){
        auto packet = macQueue.front()->dup();
        if(pkt->hasPar("riskiness")){
            packet->addPar("riskiness") = pkt->par("riskiness");
        }
        packet->setKind(BOXMAC_SUCCESS);
        cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");
        sendDirect(packet, sensorRouting, "macPackageIn");
    }
    void BOXMac::returnFailedListening(){   //send the message to routing layer that no data packets has been received after the checkIntvel time out.
        cMessage *listening_failed;
        listening_failed = new cMessage("listening_failed");
        listening_failed->setKind(BOXMAC_LISTENING_FAILED);
        cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");
        sendDirect(listening_failed, sensorRouting, "macPackageIn");
    }
    void BOXMac::initialize(int stage){
//-----debug-----------------------------
        ll = false;
        jj = false;
        executeOnce = true;
        ackCount = 0;
        globalBackoff = 0.0;
        getSimulationOn = false;
        sensorNumber = -1;
        node0State = -1;
        wakeUpTime = 0.0;
        CCAAttempt1ClearCount = 0;
        //debug log
        cumTimeToGatewayRecords = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/cumTimeToGatewayRecords.txt";

        //-------model_sim--------------
        modelSimTimeStartFileLocation = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/modelSimTimeStart.txt";
        modelSimTimeEndFileLocation = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/modelSimTimeEnd.txt";
        modelSimRadioStateFileLocation = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/radioStateValue.txt";
        stateLog = "/home/virtualforest/eclipse-workspace/virtualforest/src/results/stateLog.txt";
//-----debug end-------------------------
        offFlag=false;
        killFlag=false;
        periodicWakeupInitiated = false;
        packetReceivedDuringCCA = false;
        packetReceivedDuringINIT = false;
        consecutiveCCAAttempts = 0;
        isWakeupScheduled = false;
        isPacketBroadcast = false;
        boxmacSwitchingCount = 0;
        maxAllowedBroadcastMacSwitchingCount = 2;
        CCAAttempt1Clear = false;
        //-------Q learning related----------
        dataPacketReceived = false;
        isDistributed = false;
        isCCASleep = false;
        fire = false;
        cumulativeTimeToGateway = 0.0;
        cumulativeBackoffTime = 0.0;
        lastPacketSendTime = 0.0;
        lastMeasurementSendCount = 0;

        //-----------------------------------
        MacProtocolBase::initialize(stage);
        if (stage == INITSTAGE_LOCAL) {
            isSink = par("isSink");
            headerLength = par("AckHeaderLength");

            queueLength   = par("queueLength");

            slotDuration  = par("slotDuration");
            checkInterval = par("checkInterval");
            sleepDuration = par("sleepDuration");
//            CCAInterval = par("CCAInterval");
//            interPeriodicWakeupDuration = par("interPeriodicWakeupDuration");
            periodicWakeupDuration = par("periodicWakeupDuration");

            waitInterval = par("waitInterval");
            sendDelay = par("sendDelay");
            maximumBackOffTime = par("maximumBackOffTime");
            resolutionBackOffTime = par("resolutionBackOffTime");
            minimumBackOffTime = maximumBackOffTime/resolutionBackOffTime;
            congestion = minimumBackOffTime;   //Initialize metrics

            bitrate       = par("bitrate");
            lastDataPktDestAddr = MacAddress::BROADCAST_ADDRESS;
            lastDataPktSrcAddr  = MacAddress::BROADCAST_ADDRESS;
            macState = INIT;
            WATCH(macState);
        }else if (stage == INITSTAGE_LINK_LAYER) {
            cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
            radioModule->subscribe(IRadio::radioModeChangedSignal, this);
            radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
            radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
            radio = check_and_cast<IRadio *>(radioModule);
            initializeMacAddress();
            registerInterface();
            start_boxmac = new cMessage("start_xmac");
            start_boxmac->setKind(BOXMAC_START_BOXMAC);
            listenChannel_timeout = new cMessage("listenChannel_timeout");
            listenChannel_timeout->setKind(BOXMAC_LISTEN_CHANNEL_TIMEOUT);
            wakeup = new cMessage("wakeup");
            wakeup->setKind(BOXMAC_WAKE_UP);
            stop_preambles = new cMessage("stop_preambles");
            stop_preambles->setKind(BOXMAC_STOP_PREAMBLES);
            switch_preamble_phase = new cMessage("switch_preamble_phase");
            switch_preamble_phase->setKind(BOXMAC_SWITCH_PREAMBLE_PHASE);
            switching_done = new cMessage("switching_done");
            switching_done->setKind(BOXMAC_SWITCHING_FINISHED);
            ack_tx_over = new cMessage("ack_tx_over");
            ack_tx_over->setKind(BOXMAC_ACK_TX_OVER);
            delay_for_ack_within_remote_rx = new cMessage("delay_for_ack_within_remote_rx");
            delay_for_ack_within_remote_rx->setKind(BOXMAC_DELAY_FOR_ACK_WITHIN_REMOTE_RX);
            return_delay = new cMessage("return_delay");
            return_delay->setKind(BOXMAC_RETURN_DELAY);
            periodicWakeup = new cMessage("periodicWakeup");
            periodicWakeup->setKind(BOXMAC_PERIODIC_WAKEUP);
            scheduleAt(0.0, start_boxmac);
        }
    }
    BOXMac::~BOXMac(){
        cancelAndDelete(wakeup);
        cancelAndDelete(stop_preambles);
        cancelAndDelete(ack_tx_over);
        cancelAndDelete(listenChannel_timeout);
        cancelAndDelete(start_boxmac);
        cancelAndDelete(switch_preamble_phase);
        cancelAndDelete(delay_for_ack_within_remote_rx);
        cancelAndDelete(switching_done);
        cancelAndDelete(periodicWakeup);
    }
    void BOXMac::flushQueue(){
        MacQueue::iterator it;
        for (it = macQueue.begin(); it != macQueue.end(); ++it) {
            delete (*it);
        }
        macQueue.clear();
    }
    void BOXMac::clearQueue(){
        macQueue.clear();
    }
    void BOXMac::finish(){
    }
    void BOXMac::initializeMacAddress(){
        const char *addrstr = par("address");
        if (!strcmp(addrstr, "auto")) {
            // assign MAC address equal to node index
            int index=getParentModule()->getParentModule()->getIndex();
            int i=0;
            int ch;
            while (index>255){
                ch=index%256;
                index=floor((double)index/256);
                address.setAddressByte(i, (unsigned char)ch);
                i++;
            }
            address.setAddressByte(i, (unsigned char)index);
            i++;
            while (i<6){
                address.setAddressByte(i, (unsigned char)0);
                i++;
            }
            if (isSink){
                address.setAddressByte(3, (unsigned char)255);
            }
            //address = MacAddress::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else {
            address.setAddress(addrstr);
        }
    }
    InterfaceEntry *BOXMac::createInterfaceEntry(){
        InterfaceEntry *e = getContainingNicModule(this);
        // data rate
        e->setDatarate(bitrate);
        // generate a link-layer address to be used as interface token for IPv6
        e->setMacAddress(address);
        e->setInterfaceToken(address.formInterfaceIdentifier());
        // capabilities
        e->setMtu(par("mtu"));
        e->setMulticast(false);
        e->setBroadcast(true);
        return e;
    }
    void BOXMac::handleUpperPacket(Packet *msg){ //Routing layer add it directly to the queue
    }
    void BOXMac::sendMacAck(){
        auto ack = makeShared<BOXMacHeader>();    //Build MAC header
        ack->setSrcAddr(address);
        ack->setDestAddr(lastDataPktSrcAddr);
        ack->setChunkLength(b(headerLength));
        ack->setType(BOXMAC_ACK);
        auto packet = new Packet("BOXMacAck", ack);
        packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::boxmac);
        //------Q learning related---------------------------------

        if(true/*isDistributed && fire*/){
           packet->addPar("dataPacketReceptionTime") = dataPacketReceptionTime;

           //------------------------debug----------------------------------------------
           int MacValueMy=0;                                                            //
           int msize=address.getAddressSize();                                          //
           for (int i=0; i<msize;i++){                                                  //
               MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);         //
           }
           if(!isSink && MacValueMy == 9){
               int k = 0;
           }
           //----------------------------------------------------------------------------

           if(!isSink){
               packet->addPar("cumulativeTimeToGateway") = queuingTime + transmissionDelay + cumulativeTimeToGateway;
               packet->addPar("riskiness")=myParameterRiskiness;
           }else{
               packet->addPar("cumulativeTimeToGateway") = 0.0;
               packet->addPar("riskiness")=7.0; //We expect even the sink to add it's riskiness in the ack, but an super safe value as it's invulnerable
           }
        }
        //----------------------------------------------------------
        attachSignal(packet, simTime());
        sendDown(packet);   //Send ack package
    }
    void BOXMac::handleSelfMessage(cMessage *msg){
        char text[128];

        //========================model sim===========================================
        if(isSink && executeOnce){
//            char modelSimTimeStart [] = "311.0";
//            char modelSimTimeEnd [] = "319.0";
            char modelSimTimeStart [] = "0.0";
            char modelSimTimeEnd [] = "0.1";
            std::string radioStateValue = to_string(-1) + "_" + "SEND" +\
                    "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));

            std::ofstream myfile;
            myfile.open (modelSimTimeStartFileLocation);
            myfile << modelSimTimeStart;
            myfile.close();

            myfile.open (modelSimTimeEndFileLocation);
            myfile << modelSimTimeEnd;
            myfile.close();

            myfile.open (modelSimRadioStateFileLocation);
            myfile << radioStateValue;
            myfile.close();

            //--------model sim--------------------------------------------
            myfile.open(stateLog); // append instead of overwrite
            myfile.close();
            //-----------------------------------------------------------

            //-----cumulative time to gateway records-----------------------
            myfile.open(cumTimeToGatewayRecords);
            myfile.close();
            //--------------------------------------------------------------

//            system("python /home/virtualforest/eclipse-workspace/virtualforest/src/nodeRadio.py &");
            executeOnce = false;
        }
        //============================================================================

        if(!killFlag){
            switch (macState) {     //Works as final state machine
            case BOXMAC_RETURN_DELAY:
                returnFailedPackage();
                break;
            case INIT:
                if (msg->getKind() == BOXMAC_START_BOXMAC) {
                    setSleep();
                }

                if (msg->getKind() == BOXMAC_WAKE_UP){
                    cancelEvent(periodicWakeup);
                    cancelEvent(wakeup);
                    cancelEvent(start_boxmac);
                    macState = SLEEP;
                    scheduleAt(simTime()+ 0.00000000001, wakeup);       //This event has to be started right now. But for the functionality of the simulation, a very small value must be added.
                }
                break;
            case SLEEP:
//                //--------------address debug------------------------------------
                if(simTime() > 0){
                   int MacValueMy=0;
                   int msize=address.getAddressSize();
                   for (int i=0; i<msize;i++){
                       MacValueMy += (long long int)address.getAddressByte(i)*pow(256,i);
                   }
                   if(MacValueMy == 29){
                        if(simTime() > 12.15 and simTime() < 12.18){
                            int h=0;
                        }
                    }
                   if(MacValueMy == 0 && msg->getKind() == BOXMAC_WAKE_UP){;
                       int h = 0;
                   }
                   if(MacValueMy == 1 && msg->getKind() == BOXMAC_WAKE_UP){
                       int h = 0;
                   }
                }
//                //------------------------------------------------------------------
                if (msg->getKind() == BOXMAC_WAKE_UP) {
                   //======================model sim=============================================//
                   //-----------------------address debug----------------------------------------//
                   int MacValueMy=0;                                                             //
                   int msize=address.getAddressSize();                                           //
                   for (int i=0; i<msize;i++){                                                   //
                       MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                   }                                                                             //
                   //----------------------------------------------------------------------------//
                   std::string radioState = "NODE_RADIO_STATE";                                  //
                   std::string radioStateValue = "";                                             //
                   if(!isSink){                                                                  //
                       radioStateValue = to_string(MacValueMy) + "_" + "CCA" +\
                               "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));      //
                   }else{                                                                        //
                       radioStateValue = to_string(MacValueMy) + "_" + "CCA" +\
                               "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));      //
                   }                                                                             //
                   std::ofstream myfile;                                                         //
                   myfile.open (modelSimRadioStateFileLocation);                                 //
                   myfile << radioStateValue;                                                    //
                   myfile.close();                                                               //
                                                                                                 //
                   if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                       myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                       myfile << radioStateValue + "\n";                                         //
                       myfile.close();                                                           //
                   }                                                                             //
                   //============================================================================//

                    cancelEvent(listenChannel_timeout);
                    cancelEvent(periodicWakeup);
                    cancelEvent(start_boxmac);
                    cancelEvent(wakeup);
                    periodicWakeupInitiated = false;

                    if(!isSink){
                        if(isCCASleep){
                            isCCASleep = false;
                            consecutiveCCAAttempts++;
                            scheduleAt(simTime() + checkInterval, listenChannel_timeout);
                            //--------------Q learning related----------------------------
                            if(isDistributed && !isSink /*&& fire*/){
                                cumulativeBackoffTime += checkInterval;
                            }
                            //-----------------------------------------------------------
                        }else{
                            consecutiveCCAAttempts = 0;
                            scheduleAt(simTime() + checkInterval, listenChannel_timeout);
                        }
                    }

                    macState = CCA_ON;
                    radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);

                    //--------------Q learning related----------------------------
                    if(isDistributed /*&& fire*/){
                        listeningStartTime = atof(simTime().str().c_str());
                    }
                    //------------------------------------------------------------
                }
                if(msg->getKind() == BOXMAC_PERIODIC_WAKEUP){
                    radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
                    cancelEvent(periodicWakeup);
                    cancelEvent(start_boxmac);
                    scheduleAt(simTime() + periodicWakeupDuration, start_boxmac);
                    periodicWakeupInitiated = true;
                    macState = INIT;

                   //===================model sim======================================//
                   //-----------------------address debug------------------------------//
                   int MacValueMy=0;                                                   //
                   int msize=address.getAddressSize();                                 //
                   for (int i=0; i<msize;i++){                                         //
                       MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);//
                   }                                                                   //
                   //------------------------------------------------------------------//
                   std::string radioState = "NODE_RADIO_STATE";                        //
                   std::string radioStateValue = to_string(MacValueMy) + "_" + "SLEEP" +\
                           "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));//
                   std::ofstream myfile;                                               //
                   myfile.open (modelSimRadioStateFileLocation);                       //
                   myfile << radioStateValue;                                          //
                   myfile.close();                                                     //
                                                                                       //
                   if(simTime() > 5.0 && simTime() < 1400.0){                          //
                       myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                       myfile << radioStateValue + "\n";                               //
                       myfile.close();                                                 //
                   }                                                                   //
                   //==================================================================//
                }
                break;

            case CCA_ON: //Check channel state
                cancelEvent(listenChannel_timeout);
                if (msg->getKind() == BOXMAC_LISTEN_CHANNEL_TIMEOUT){

                    //--------------Q learning related------------------------
                    if(!dataPacketReceived){ //&& isDistributed && !isSink && !isCCASleep /*&& fire*/){    //No data packets has been received after the checkInterval(listening time) has been timed out, and if distributed algo is activated
                        isCCASleep = false;
                        returnFailedListening();
                    }else{
                        dataPacketReceived = false;
                    }
                    //--------------------------------------------------------

                    if (receptionState == IRadio::RECEPTION_STATE_RECEIVING || receptionState == IRadio::RECEPTION_STATE_BUSY){//channel busy
                        cancelEvent(listenChannel_timeout);
                        if(!isSink){
                            if(consecutiveCCAAttempts < 1){
                                isCCASleep = true;
                            }else{
                                isCCASleep = false;
                            }
                        }
                         //=======================model sim===========================================//
                         //----------------------address debug----------------------------------------//
                         int MacValueMy=0;                                                            //
                         int msize=address.getAddressSize();                                          //
                         for (int i=0; i<msize;i++){                                                  //
                             MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);         //
                         }                                                                            //
                         //---------------------------------------------------------------------------//
                         std::string radioState = "NODE_RADIO_STATE";                                 //
                         std::string radioStateValue = to_string(MacValueMy) + "_" +
                                 to_string(atof(simTime().str().c_str())) + "_" + to_string(receptionState);
                         std::ofstream myfile;                                                        //
                         myfile.open (modelSimRadioStateFileLocation);                                //
                         myfile << radioStateValue;                                                   //
                         myfile.close();                                                              //
                                                                                                      //
                         if(simTime() > 5.0 && simTime() < 1400.0){                                   //
                             myfile.open(stateLog, std::ios_base::app); // append instead of overwrite//
                             myfile << radioStateValue + "\n";                                        //
                             myfile.close();                                                          //
                         }                                                                            //
                         //===========================================================================//

                        CCAAttempt1Clear = false;
                        CCAAttempt1ClearCount = 0;
                        setSleep();
//                        if(isSink){
//                            scheduleAt(simTime() + checkInterval, listenChannel_timeout);  //Stay in the CCA state for more time because the SN is receiving something
//                        }else{
//                            setSleep();
//                        }
//                    }else if(!CCAAttempt1Clear){
                    }else if(CCAAttempt1ClearCount < 10){
                        CCAAttempt1Clear = true;
                        CCAAttempt1ClearCount++;
                        scheduleAt(simTime() + waitInterval/8, listenChannel_timeout);
                        //=======================model sim==============================================//
                           //----------------------address debug----------------------------------------//
                          int MacValueMy=0;                                                             //
                          int msize=address.getAddressSize();                                           //
                          for (int i=0; i<msize;i++){                                                   //
                              MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                          }                                                                             //
                          //----------------------------------------------------------------------------//
                          std::string radioState = "NODE_RADIO_STATE";                                  //
                          std::string radioStateValue = to_string(MacValueMy) + "_" +
                                  to_string(atof(simTime().str().c_str())) + "_" + to_string(receptionState);
                          std::ofstream myfile;                                                         //
                          myfile.open (modelSimRadioStateFileLocation);                                 //
                          myfile << radioStateValue;                                                    //
                          myfile.close();                                                               //
                                                                                                        //
                          if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                              myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                              myfile << radioStateValue + "\n";                                         //
                              myfile.close();                                                           //
                          }                                                                             //
                          //============================================================================//

                    }else if(CCAAttempt1ClearCount < 11){
                        CCAAttempt1ClearCount++;
                        scheduleAt(simTime() + uniform(0,waitInterval/8), listenChannel_timeout);

                        //=======================model sim===================================================//
                           //----------------------address debug---------------------------------------------//
                          int MacValueMy=0;                                                                  //
                          int msize=address.getAddressSize();                                                //
                          for (int i=0; i<msize;i++){                                                        //
                              MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);               //
                          }                                                                                  //
                          //---------------------------------------------------------------------------------//
                          std::string radioState = "NODE_RADIO_STATE";                                       //
                          std::string radioStateValue = to_string(MacValueMy) + "_" +
                                  to_string(atof(simTime().str().c_str())) + "_" + to_string(receptionState);//
                          std::ofstream myfile;                                                              //
                          myfile.open (modelSimRadioStateFileLocation);                                      //
                          myfile << radioStateValue;                                                         //
                          myfile.close();                                                                    //
                                                                                                             //
                          if(simTime() > 5.0 && simTime() < 1400.0){                                         //
                              myfile.open(stateLog, std::ios_base::app); // append instead of overwrite      //
                              myfile << radioStateValue + "\n";                                              //
                              myfile.close();                                                                //
                          }                                                                                  //
                          //=================================================================================//

                    }else {//!isSink part is added by Nisal to prevent the gateway transition to the SEND_DATA state
                        CCAAttempt1Clear = false;
                        CCAAttempt1ClearCount = 0;

                         //=======================model sim============================================//
                         //----------------------address debug-----------------------------------------//
                         int MacValueMy=0;                                                             //
                         int msize=address.getAddressSize();                                           //
                         for (int i=0; i<msize;i++){                                                   //
                             MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                         }                                                                             //
                         //----------------------------------------------------------------------------//
                         std::string radioState = "NODE_RADIO_STATE";                                  //
                         std::string radioStateValue = to_string(MacValueMy) + "_" +
                                 to_string(atof(simTime().str().c_str())) + "_" + to_string(receptionState);
                         std::ofstream myfile;                                                         //
                         myfile.open (modelSimRadioStateFileLocation);                                 //
                         myfile << radioStateValue;                                                    //
                         myfile.close();                                                               //
                                                                                                       //
                         if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                             myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                             myfile << radioStateValue + "\n";                                         //
                             myfile.close();                                                           //
                         }                                                                             //
                         //============================================================================//

                        if (macQueue.size() > 0){   //Check if I have packages to send
                            macState = SEND_DATA;
                            radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                            scheduleAt(simTime() + slotDuration, stop_preambles);

                            //=======================model sim===========================================//
                            //----------------------address debug----------------------------------------//
                           int MacValueMy=0;                                                             //
                           int msize=address.getAddressSize();                                           //
                           for (int i=0; i<msize;i++){                                                   //
                               MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                           }                                                                             //
                           //----------------------------------------------------------------------------//
                           std::string radioState = "NODE_RADIO_STATE";                                  //
                           std::string radioStateValue = to_string(MacValueMy) + "_" + "SEND" +\
                                   "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));          //
                           std::ofstream myfile;                                                         //
                           myfile.open (modelSimRadioStateFileLocation);                                 //
                           myfile << radioStateValue;                                                    //
                           myfile.close();                                                               //
                                                                                                         //
                           if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                               myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                               myfile << radioStateValue + "\n";                                         //
                               myfile.close();                                                           //
                           }                                                                             //
                           //============================================================================//

                        }else{
                            setSleep();
                        }
                    }
                }
                break;

            case SEND_DATA:
                if (msg->getKind() == BOXMAC_SWITCH_PREAMBLE_PHASE) {  //Process to switch between sending a data package and waiting for an ack package until the slot time is over
                    if (radio->getRadioMode() == IRadio::RADIO_MODE_RECEIVER) {

                        //=======================model sim===========================================//
                        //----------------------address debug----------------------------------------//
                       int MacValueMy=0;                                                             //
                       int msize=address.getAddressSize();                                           //
                       for (int i=0; i<msize;i++){                                                   //
                           MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                       }                                                                             //
                       //----------------------------------------------------------------------------//
                       std::string radioState = "NODE_RADIO_STATE";                                  //
                       std::string radioStateValue = to_string(MacValueMy) + "_" + "SEND" +\
                               "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));          //
                       std::ofstream myfile;                                                         //
                       myfile.open (modelSimRadioStateFileLocation);                                 //
                       myfile << radioStateValue;                                                    //
                       myfile.close();                                                               //
                                                                                                     //
                       if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                           myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                           myfile << radioStateValue + "\n";                                         //
                           myfile.close();                                                           //
                       }                                                                             //
                       //============================================================================//

//                        if (receptionState == IRadio::RECEPTION_STATE_RECEIVING || receptionState == IRadio::RECEPTION_STATE_BUSY){//channel busy

                        //--------Q learning related---------------------------------
                        if(!dataPacketReceived && isDistributed && !isSink /*&& fire*/){    //No data packets has been received after the checkInterval(listening time) has been timed out, and if distributed algo is activated
                            returnFailedListening();
                        }else{
                            dataPacketReceived = false;
                        }
                        //-----------------------------------------------------------
                        radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                        if(isPacketBroadcast)
                        {
                            if(boxmacSwitchingCount < maxAllowedBroadcastMacSwitchingCount){ //Added by Nisal to restrict the number of packet broadcasting transmission and listening attempts to 3
                                boxmacSwitchingCount++;
                            } else {
                                isPacketBroadcast = false;
                                boxmacSwitchingCount = 0;
                                cancelEvent(stop_preambles);
                                scheduleAt(simTime() + 0.0000000001, stop_preambles);
                            }
                        }
                    }else if (radio->getRadioMode() == IRadio::RADIO_MODE_TRANSMITTER) {

                        radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
                        //--------------Q learning related----------------------------
                        if(isDistributed /*&& fire*/){
                            listeningStartTime = atof(simTime().str().c_str());
                        }
                        //------------------------------------------------------------

                        //=======================model sim===========================================//
                        //----------------------address debug----------------------------------------//
                       int MacValueMy=0;                                                             //
                       int msize=address.getAddressSize();                                           //
                       for (int i=0; i<msize;i++){                                                   //
                           MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);          //
                       }                                                                             //
                       //----------------------------------------------------------------------------//
                       std::string radioState = "NODE_RADIO_STATE";                                  //
                       std::string radioStateValue = to_string(MacValueMy) + "_" + "SEND" +\
                               "_" + "rx" + "_" + to_string(atof(simTime().str().c_str()));          //
                       std::ofstream myfile;                                                         //
                       myfile.open (modelSimRadioStateFileLocation);                                 //
                       myfile << radioStateValue;                                                    //
                       myfile.close();                                                               //
                                                                                                     //
                       if(simTime() > 5.0 && simTime() < 1400.0){                                    //
                           myfile.open(stateLog, std::ios_base::app); // append instead of overwrite //
                           myfile << radioStateValue + "\n";                                         //
                           myfile.close();                                                           //
                       }                                                                             //
                       //============================================================================//

                        scheduleAt(simTime() + waitInterval, switch_preamble_phase);
                    }
                }else if (msg->getKind() == BOXMAC_SWITCHING_FINISHED) {
                            sendDataPacket();

//                    //=======================model sim===========================================
//                    //----------------------address debug----------------------------------------
//                   int MacValueMy=0;
//                   int msize=address.getAddressSize();
//                   for (int i=0; i<msize;i++){
//                       MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
//                   }
//                   if(isSink){
////                       char text[128];
////                       sprintf(text," sensorNumber: %d ", MacValueMy);
////                       getSimulation()->getActiveEnvir()->alert(text);
//                       MacValueMy = -2;
//                   }
//                   //-----------------------------------------------------------------------------
//                   std::string radioState = "NODE_RADIO_STATE";
//                   std::string radioStateValue = to_string(MacValueMy) + "_" + "SEND" +\
//                           "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));
//                   std::ofstream myfile;
//                   myfile.open (modelSimRadioStateFileLocation);
//                   myfile << radioStateValue;
//                   myfile.close();
//
//                   if(simTime() > 5.0 && simTime() < 1400.0){
//                       myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
//                       myfile << radioStateValue + "\n";
//                       myfile.close();
//                   }

                    //--------------Q learning related----------------------------
                    if(true/*isDistributed && fire*/){
                        lastPacketRetransmissionTime = atof(simTime().str().c_str());
                        retransmissionCount++;
                    }
                    //------------------------------------------------------------
                }else if (msg->getKind() == BOXMAC_STOP_PREAMBLES) {  //If it did not received an ack package, the MAC layer resend the data package to the routing layer
                    if (lastDataPktDestAddr!=MacAddress::BROADCAST_ADDRESS){
                        returnFailedPackage();
                        if(isDistributed /*&& fire*/){
                            retransmissionCount = 0;
                        }
                    }
                    delete macQueue.front();
                    macQueue.pop_front();
                    cancelEvent(listenChannel_timeout);
                    cancelEvent(stop_preambles);
                    cancelEvent(switch_preamble_phase);
                    setSleep();
                }
                break;
            case SEND_ACK:
                //-------------------------debug-----------------------------------
                if(isSink){
                   std::ofstream myfile;
                   string ackFileDirectory = "/home/virtualforest/omnetpp-workspace/inet4/examples/sensornetwork/recordings/";
                   myfile.open (ackFileDirectory + "gatewayACKSent" + to_string( ackCount/1024)+ ".csv", std::ios_base::app);
                   ackCount++;
                   myfile << atof(simTime().str().c_str()) << ",";
                   myfile.close();
                }
                //----------------------------------------------------------------

                if (msg->getKind() == BOXMAC_DELAY_FOR_ACK_WITHIN_REMOTE_RX) {
                    sendMacAck();
                    macState = WAIT_ACK_TX;
                }
                break;

            case WAIT_ACK_TX:
                if (msg->getKind() == BOXMAC_ACK_TX_OVER) {
                    if(isWakeupScheduled != wakeup->isScheduled()){
                        periodicWakeupInitiated = false;
                        isWakeupScheduled = false;
                    }

                    if(isSink){
//                        packetReceivedDuringCCA = false;
                        cancelEvent(listenChannel_timeout);
//                        changeCongestion(1); //Increase congestion
                        setSleep();
                        packetReceivedDuringCCA = false;
//                        double backofftime=uniform(0,congestion);
//                        radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
//                        macState = CCA_ON;
//                        scheduleAt(simTime()+backofftime+checkInterval, listenChannel_timeout);
                    }

                    if(packetReceivedDuringINIT){
                        packetReceivedDuringINIT = false;
                        macState = INIT;
                        setSleep();

                    }else if(packetReceivedDuringCCA && !isSink){
                        packetReceivedDuringCCA = false;
                        setBackOff();
                    }else if(isSink){
//                        setSleep();
//                        getSimulation()->getActiveEnvir()->alert("sink ack_2");
//                        cancelEvent(listenChannel_timeout);
//                        double backofftime=uniform(0,congestion);
//                        radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
//                        macState = CCA_ON;
////                        scheduleAt(simTime()+backofftime+checkInterval, listenChannel_timeout);
//                        scheduleAt(simTime()+0.000001, listenChannel_timeout);
                    }
                }

                break;
            }
        }else{
            setOff();
        }
        //changeDisplayState();
    }
    void BOXMac::handleLowerPacket(Packet *packet){
        if (!killFlag){
            if (macState==CCA_ON && packet->hasBitError()){
                cancelEvent(listenChannel_timeout);
                scheduleAt(simTime() + checkInterval, listenChannel_timeout); //Stay in the CCA state for more time because maybe the package was for me
            }
            if (packet->hasBitError()) {
                delete packet;
            }else{
                const auto& mac = packet->peekAtFront<BOXMacHeader>();
                switch (macState) {
                case INIT:
                {
                    if (mac->getType() == BOXMAC_DATA) {
                        cancelEvent(listenChannel_timeout);

                        if (mac->getDestAddr() == address || mac->getDestAddr().isBroadcast()) { //It received a meassurments data package or neighbor parameters
                            cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");

                            if (mac->getDestAddr() == address){ //Received neighbor meassurments
                                packetReceivedDuringINIT = true;
                                if(!isSink){
                                    //=======================model sim===========================================
                                    //----------------------address debug----------------------------------------
                                   int MacValueMy=0;
                                   int msize=address.getAddressSize();
                                   for (int i=0; i<msize;i++){
                                       MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
                                   }
                                   //-----------------------------------------------------------------------------
                                   std::string radioState = "NODE_RADIO_STATE";
                                   std::string radioStateValue = to_string(MacValueMy) + "_" + "ACK" +\
                                           "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));
                                   std::ofstream myfile;
                                   myfile.open (modelSimRadioStateFileLocation);
                                   myfile << radioStateValue;
                                   myfile.close();

                                   if(simTime() > 5.0 && simTime() < 1400.0){
                                       myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                                       myfile << radioStateValue + "\n";
                                       myfile.close();
                                   }
                                   //=============================================================================
                                }

                                lastDataPktSrcAddr = mac->getSrcAddr();
                                macState = SEND_ACK;
                                radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                                isWakeupScheduled = wakeup->isScheduled();

                                //--------------Q learning related----------------------------
                                if(true/*isDistributed && fire*/){
                                    dataPacketReceived = true;
                                    dataPacketReceptionTime = atof(simTime().str().c_str());
                                }
                                //------------------------------------------------------------

                                sendDirect(packet, sensorRouting, "macPackageIn");
                            }else{  //It received a neighbor discovery data package
                                sendDirect(packet, sensorRouting, "macPackageIn");
                            }
                        }else { //The package is not for me
                            delete(packet);
                        }
                    }else{
                        delete(packet); //ACK probably
                    }
                    break;
                }
                case CCA_ON:
                {
                    if(!isSink){
                        if(simTime() > 1076.74){
                            int k = 0;
                        }
                        int h = 0;
                    }
                    char text[128];
                    if (mac->getType() == BOXMAC_DATA) {
                        cancelEvent(listenChannel_timeout);
                            if (mac->getDestAddr() == address || mac->getDestAddr().isBroadcast()) { //It received a meassurments data package or neighbor parameters
                                                        cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");
                                if (mac->getDestAddr() == address){ //Received neighbor meassurments
    //                                ackCount += 1;
                                    packetReceivedDuringCCA = true;
    //                                ll = true;
                                    macState = SEND_ACK;
                                    lastDataPktSrcAddr = mac->getSrcAddr();
                                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                                        //=======================model sim===========================================
                                        //----------------------address debug----------------------------------------
                                       int MacValueMy=0;
                                       int msize=address.getAddressSize();
                                       for (int i=0; i<msize;i++){
                                           MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
                                       }
                                       //-----------------------------------------------------------------------------
                                       std::string radioState = "NODE_RADIO_STATE";
                                       std::string radioStateValue = to_string(MacValueMy) + "_" + "ACK" +\
                                               "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));
                                       std::ofstream myfile;
                                       myfile.open (modelSimRadioStateFileLocation);
                                       myfile << radioStateValue;
                                       myfile.close();

                                       if(simTime() > 5.0 && simTime() < 1400.0){
                                           myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                                           myfile << radioStateValue + "\n";
                                           myfile.close();
                                       }
                                       //=============================================================================

                                    //--------------Q learning related----------------------------
                                    if(true/*isDistributed && fire*/){
                                        dataPacketReceived = true;
                                        dataPacketReceptionTime = atof(simTime().str().c_str());
                                    }
                                    //------------------------------------------------------------
                                    sendDirect(packet, sensorRouting, "macPackageIn");
                                }else{  //It received a neighbor discovery data package
                                    sendDirect(packet, sensorRouting, "macPackageIn");
                                }
                            }else { //The package is not for me
                                delete(packet);
                            }
//                        }

                    }else{
                        delete(packet); //ACK probably
                    }
                    setBackOff();
                    break;
                }
                case SEND_DATA:
                    if (mac->getType() == BOXMAC_DATA) {
                        if (mac->getDestAddr() == address || mac->getDestAddr().isBroadcast()) { //It received a meassurments data package or neighbor parameters
                            cModule *sensorRouting = getParentModule()->getParentModule()->getSubmodule("sensorRouting");
                            if (mac->getDestAddr() == address){                         //Received neighbor meassurments
                                //=======================model sim===========================================
                                //----------------------address debug----------------------------------------
                               int MacValueMy=0;
                               int msize=address.getAddressSize();
                               for (int i=0; i<msize;i++){
                                   MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
                               }
                               //-----------------------------------------------------------------------------
                               std::string radioState = "NODE_RADIO_STATE";
                               std::string radioStateValue = to_string(MacValueMy) + "_" + "ACK" +\
                                       "_" + "tx" + "_" + to_string(atof(simTime().str().c_str()));
                               std::ofstream myfile;
                               myfile.open (modelSimRadioStateFileLocation);
                               myfile << radioStateValue;
                               myfile.close();

                               if(simTime() > 5.0 && simTime() < 1400.0){
                                   myfile.open(stateLog, std::ios_base::app); // append instead of overwrite
                                   myfile << radioStateValue + "\n";
                                   myfile.close();
                               }
                               //=============================================================================

//                              ackCount += 1;
                                macState = SEND_ACK;
                                lastDataPktSrcAddr = mac->getSrcAddr();

                                //--------------Q learning related----------------------------
                                if(true/*isDistributed && fire*/){
                                    dataPacketReceived = true;
                                    dataPacketReceptionTime = atof(simTime().str().c_str());
                                }
                                //------------------------------------------------------------
                                macState = SEND_ACK;
                                lastDataPktSrcAddr = mac->getSrcAddr();
                                radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                                sendDirect(packet, sensorRouting, "macPackageIn");
                                cancelEvent(stop_preambles);
                                cancelEvent(switch_preamble_phase);
                                cancelEvent(listenChannel_timeout);
                                scheduleAt(simTime(),return_delay);                     //returnFailedPackage();
                            }else{                                                      //It received a neighbor discovery data package
                                sendDirect(packet, sensorRouting, "macPackageIn");
                            }

                        }else {                                                         //The package is not for me
                            delete(packet);
                        }

                    }else if (mac->getType() == BOXMAC_ACK) {                           //It receive an ack package after sending a data package
                        if (mac->getDestAddr() == address && mac->getSrcAddr() == lastDataPktDestAddr){

                            //--------------Q learning related----------------------------
                            ackReceptionTime = atof(simTime().str().c_str());
                            //--------------------debug-----------------------------------------

                            //=======================model sim============================================//
                              //----------------------address debug---------------------------------------//
                             int MacValueMy=0;                                                            //
                             int msize=address.getAddressSize();                                          //
                             for (int i=0; i<msize;i++){                                                  //
                                 MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);         //
                             }                                                                            //
                             //---------------------------------------------------------------------------//
                             std::string radioState = "NODE_RADIO_STATE";                                 //
                             std::string radioStateValue = to_string(MacValueMy) + "_" + to_string(atof(simTime().str().c_str()));
                             std::ofstream myfile;                                                        //
                             myfile.open (modelSimRadioStateFileLocation);                                //
                             myfile << radioStateValue;                                                   //
                             myfile.close();                                                              //
                                                                                                          //
                             if(simTime() > 5.0 && simTime() < 1400.0){                                   //
                                 myfile.open(stateLog, std::ios_base::app); // append instead of overwrite//
                                 myfile << radioStateValue + "\n";                                        //
                                 myfile.close();                                                          //
                             }                                                                            //
                           //=============================================================================//

                            //------------------------------------------------------------------
                            if(true/*isDistributed && fire*/){  //remove the true
                                double dataPacketReceptionTimeAtNeighbor;

                                if(packet->hasPar("dataPacketReceptionTime")){
                                    dataPacketReceptionTimeAtNeighbor = packet->par("dataPacketReceptionTime");//ackReceptionTime becomes the time the parent has received the packet
                                }

                                if(packet->hasPar("cumulativeTimeToGateway")){
                                    cumulativeTimeToGateway = packet->par("cumulativeTimeToGateway");
                                }

                                transmissionDelay = (dataPacketReceptionTimeAtNeighbor - lastPacketRetransmissionTime); //This only one way trip time.

                                //=======================cumTimeToGatewayRecords log debug==================//
                                //----------------------address debug---------------------------------------//
                               int MacValueMy=0;                                                            //
                               int msize=address.getAddressSize();                                          //
                               for (int i=0; i<msize;i++){                                                  //
                                   MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);         //
                               }
                               if(!isSink && MacValueMy == 9){
                                   int k = 0;
                               }
//                               if(packet->hasPar("cumulativeTimeToGateway")){
//                                   cumulativeTimeToGateway = packet->par("cumulativeTimeToGateway");
//                               }
//                               double transmissionDelay = (dataPacketReceptionTimeAtNeighbor - lastPacketRetransmissionTime);
//                               cumulativeTimeToGateway =  cumulativeTimeToGateway + transmissionDelay;
                               //---------------------------------------------------------------------------//
                                if(!isSink && MacValueMy == 3){                                             //
                                   std::ofstream myfile;                                                    //
                                   std::string cumTimeToGateway = to_string(MacValueMy)
                                           +"_cumTime_"+ to_string(cumulativeTimeToGateway);
                                   std::string transmissionDel = to_string(MacValueMy)
                                           +"_transDelay_"+ to_string(transmissionDelay); //
                                   myfile.open(cumTimeToGatewayRecords, std::ios_base::app); // append instead of overwrite
                                   myfile << cumTimeToGateway + "\n";
                                   myfile << transmissionDel + "\n"; //
                                   myfile.close();                                                          //
                                }                                                                           //
                                //========================cumTimeToGatewayRecords debug end=================//
                            }
                            //------------------------------------------------------------
                            returnSuccessPackage(packet);
                            delete macQueue.front();
                            macQueue.pop_front();
                            cancelEvent(listenChannel_timeout);
                            cancelEvent(stop_preambles);
                            cancelEvent(switch_preamble_phase);
                            setSleep();
                            delete(packet); //ACK
                        }
                    }else{
                        delete(packet); //ACK
                    }

                    break;
                default:
                    delete(packet);
                }
                //changeDisplayState();
            }
        }else{
            setOff();
        }
    }
    void BOXMac::sendDataPacket(){ //Send package from queue
        auto packet = macQueue.front()->dup();
        const auto& hdr = packet->peekAtFront<BOXMacHeader>();
        lastDataPktDestAddr = hdr->getDestAddr();
        //----added by Nisal----------------------
        isPacketBroadcast = lastDataPktDestAddr.isBroadcast();
        //-----------------------------------------
        //-------Q learning related------------------------------
        if(!isPacketBroadcast){
//            //-----------------------address debug-------------------------------------------
            double timeStamp = atof(simTime().str().c_str());
            int queueSize = macQueue.size();
            int MacValueMy=0;
            int msize=address.getAddressSize();
            for (int i=0; i<msize;i++){
                MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
            }
            if(MacValueMy == 9){
                int g = 0;
            }
//            //-----------------address debug end-------------------------------------------
            int measurementSize  = 32;   //the size of a measurement is 32 bits
            int packetMacHeaderLength = 64; //from the omnetpp.ini, the length of the MAC header is set to 64b
            lastMeasurementSendCount = (packet->getBitLength() - packetMacHeaderLength)/measurementSize; //number of data chunks (measurements) consisted in the packet lastly sent
            lastPacketSendTime = atof(simTime().str().c_str()); //time stamp of the transmission of the packet, lastly dispatched from the MAC layer
        }
        //------------------------------------------------------
        attachSignal(packet, simTime());
        sendDown(packet);

        // debug--------------------------------------------------
//        char text[128];
//        sprintf(text,"sending measurementCount: %d, send time: %f", packet->getDataLength()/measurementSize, lastPacketSendTime);
//        getSimulation()->getActiveEnvir()->alert(text);

        if(isSink){
//            if(simTime() >= 315.715){
                //-----------------------address debug-------------------------------------------
//                int MacValueMy=0;
//                int msize=address.getAddressSize();
//                for (int i=0; i<msize;i++){
//                    MacValueMy+=(long long int)address.getAddressByte(i)*pow(256,i);
//                }
//                if(MacValueMy == 0.0){
//                    int g = 0;
//                }
//
//                int MacValueParent=0;
//                msize=lastDataPktDestAddr.getAddressSize();
//                for (int i=0; i<msize;i++){
//                    MacValueParent+=(long long int)lastDataPktDestAddr.getAddressByte(i)*pow(256,i);
//                }
//                double kk = atof(simTime().str().c_str());
//                int y = 0;
    ////
//                char text[128];
    ////            sprintf(text,"SEND_DATA sendDataPacket: %d to %d : %f", MacValueMy,MacValueParent, atof(simTime().str().c_str()));
    ////            if(getSimulationOn){
    ////                if(atof(simTime().str().c_str()) >  410.816000){
    ////                    getSimulation()->getActiveEnvir()->alert(text);
    ////                }
    ////            }
                //-----------------------------------------------------------------------
//            }
        }

        // debug end----------------------------------------------------
    }
    void BOXMac::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details){
        Enter_Method_Silent();
        if (!killFlag){
            if (signalID == IRadio::receptionStateChangedSignal) {
                receptionState = (IRadio::ReceptionState)value;

                //--------------------address debug-----------------------------
                int MacValueMy=0;
                int msize=address.getAddressSize();
                for (int i=0; i<msize;i++){
                    MacValueMy += (long long int)address.getAddressByte(i)*pow(256,i);
                }
                if(MacValueMy == 29){
                     if(simTime() > 12.15 and simTime() < 12.18){
                         int h=0;
                     }
                 }
                //-------------------------------------------------------------

                if (macState==CCA_ON){
                    if (receptionState == IRadio::RECEPTION_STATE_RECEIVING || receptionState == IRadio::RECEPTION_STATE_BUSY){ //Extend time in CCA estate because activity in the radio input
                        cancelEvent(listenChannel_timeout);
                        scheduleAt(simTime() + checkInterval, listenChannel_timeout);
                    }
                }
            }
            if (signalID == IRadio::transmissionStateChangedSignal) {
                IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
                if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
                    // Transmission of one packet is over
                    if (macState == SEND_DATA) {
                        scheduleAt(simTime(), switch_preamble_phase);
                    }
                    if (macState == WAIT_ACK_TX) {
                        scheduleAt(simTime(), ack_tx_over);
                    }
                }
                transmissionState = newRadioTransmissionState;
            }
            else if (signalID ==IRadio::radioModeChangedSignal) {
                // Radio switching (to RX or TX) is over, ignore switching to SLEEP.
                if (radio->getRadioMode() == IRadio::RADIO_MODE_TRANSMITTER) {
                    if (macState == SEND_ACK) {
                        scheduleAt(simTime()+sendDelay, delay_for_ack_within_remote_rx);
                    }
                    else if (macState == SEND_DATA) {
                        cancelEvent(switching_done);
                        scheduleAt(simTime()+sendDelay, switching_done);
                    }
                }
            }
        }else{
            setOff();
        }
    }
    void BOXMac::attachSignal(Packet *packet, simtime_t_cref startTime){
        simtime_t duration = packet->getBitLength() / bitrate;
        packet->setDuration(duration);
    }
    void BOXMac::changeDisplayState(){    //Show SN MAC state
        cDisplayString& dispStr = getContainingNode(this)->getDisplayString();
        switch (macState) {
            case INIT:
                dispStr.setTagArg("t", 0, "INIT");
                break;
            case SLEEP:
                dispStr.setTagArg("t", 0, "SLEEP");
                break;
            case CCA_ON:
                dispStr.setTagArg("t", 0, "CCA_ON");
                break;
            case SEND_ACK:
                dispStr.setTagArg("t", 0, "SEND_ACK");
                break;
            case SEND_DATA:
                dispStr.setTagArg("t", 0, "SEND_DATA");
                break;
            case WAIT_ACK_TX:
                dispStr.setTagArg("t", 0, "WAIT_ACK_TX");
                class MacPkt;
        }
    }
} // namespace inet

// -----steps to debug at runtime-----
// char text[128];
// sprintf(text,"ackCount: %d", ackCount);
// getSimulation()->getActiveEnvir()->alert(text);
