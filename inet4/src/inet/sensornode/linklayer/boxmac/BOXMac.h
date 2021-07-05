#ifndef __INET_BOXMAC_H_
#define __INET_BOXMAC_H_

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <cmath>

#include "inet/common/INETDefs.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/linklayer/contract/IMacProtocol.h"
#include "BOXMacHeader_m.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/sensornode/structures/structures.h"

namespace inet {
    class MacPkt;
    class INET_API BOXMac : public MacProtocolBase, public IMacProtocol{
      private:
        BOXMac(const BOXMac&);
        BOXMac& operator=(const BOXMac&);
      public:
        BOXMac()
            : MacProtocolBase()
            , macQueue()
            , macState(INIT)
            , start_boxmac(NULL), wakeup(NULL)
            , ccaON_timeout(NULL), ccaOFF_timeout(NULL), listenChannel_timeout(NULL)
            , ack_tx_over(NULL), stop_preambles(NULL), return_delay(NULL), periodicWakeup(NULL)
            , lastDataPktSrcAddr()
            , lastDataPktDestAddr()
            , queueLength(0)
            , slotDuration(0), bitrate(0), checkInterval(0)
        {}
        virtual ~BOXMac();
        /** @brief Initialization of the module and some variables*/
        virtual void initialize(int) override;
        /** @brief Delete all dynamically allocated objects of the module*/
        virtual void finish() override;
        /** @brief Handle messages from lower layer */
        virtual void handleLowerPacket(Packet *) override;
        /** @brief Handle messages from upper layer */
        virtual void handleUpperPacket(Packet *) override;
        /** @brief Handle self messages such as timers */
        virtual void handleSelfMessage(cMessage *) override;
        void receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details) override;
        //Set times to build the bahavior of rdc
        void setSlotDuration(double value);     //Time for sending copies of a package
        void setCheckInterval(double value);    //Time for listening the channel before deciding if is free or bussy
        void setWaitInterval(double value);     //Time for waiting an aknowledgment package
        void setSendDelay(double value);        //Time before sending a package
        void setSleepDuration(double value);    //Time for turning off the radio
        //Get SN parameters
        double getCongestion();
        int getMaxMacQueSize();
        int getMacQueSize();
        bool isDead();
        MacAddress getMacLayerAddress();
        double getWaitInterval();
        //Funtions to activate or deactivate a SN
        void setOff();
        void setOn();
        void setDead();
        //Funtion to send something from the routing layer to the MAC layer
        void sendToMAClayer(Packet * packet);
        //Functions for back off
        virtual void setBackOff();
        virtual void setSleep();
        virtual void changeCongestion(int direction);
        virtual void returnFailedPackage();
        virtual void returnSuccessPackage(Packet * packet);

        //----------Q learning related data-------------------
        void returnFailedListening(); //when no data packets received during the listening state
        double getAckReceptionTime();
        void resetAckReceptionTime();
        double getLastPacketRetransmissionTime();
        void resetLastPacketRetransmissionTime();
        int getRetransmissionCount();
        void resetRetransmissionCount();
        double getListeningStartTime();
        void resetListeningStartTime();
        double getDataPacketReceptionTime();
        void resetDataPacketReceptionTime();
        void setMyParameterRiskiness(double value);
        void setIsDistributed(bool value);
        double getCumulativeBackoffTime();
        void resetCumulativeBackofftime();
        double getCumulativeTimeToGateway();
        void setQueuingTime(double value);
        double getTransmissionDelay();
        void setFire();
        double getLastPacketSentTime(); // the time the process of the sending of the last packet was started
        int getLastPacketMeasurementCount(); // get the number of measurement contained in the last packet sent

        //---------------debug-----------------------------
        int getSensorNumber();
        double getSlotDuration();
        double getCheckInterval();


      protected:
        typedef std::list<Packet *> MacQueue;
        virtual void flushQueue();
        virtual void clearQueue();
        virtual InterfaceEntry *createInterfaceEntry() override;
        virtual void initializeMacAddress();
        MacQueue macQueue;
        enum States {
            INIT,           //0
            SLEEP,          //1
            SEND_DATA,      //2
            SEND_ACK,       //3
            WAIT_ACK_TX,    //4
            CCA_ON,         //5
          };
        States macState;
        // messages used in the FSM
        cMessage *start_boxmac;
        cMessage *wakeup;
        cMessage *ccaON_timeout;
        cMessage *ccaOFF_timeout;
        cMessage *listenChannel_timeout;
        cMessage *ack_tx_over;
        cMessage *switch_preamble_phase;
        cMessage *delay_for_ack_within_remote_rx;
        cMessage *stop_preambles;
        cMessage *switching_done;
        cMessage *return_delay;
        cMessage *periodicWakeup;

        /** @name Help variables for the acknowledgment process. */
        MacAddress lastDataPktSrcAddr;
        MacAddress lastDataPktDestAddr;
        int headerLength = 0;    // BOXMacFrame header length in bytes
        MacAddress address;    // MAC address
        /** @brief The radio. */
        physicallayer::IRadio *radio;
        physicallayer::IRadio::TransmissionState transmissionState = physicallayer::IRadio::TRANSMISSION_STATE_UNDEFINED;
        physicallayer::IRadio::IRadio::ReceptionState receptionState = physicallayer::IRadio::RECEPTION_STATE_UNDEFINED;
        /** @brief The maximum length of the queue */
        unsigned int queueLength;

        double slotDuration;            //Time for transmiting data
        double waitInterval;            //Time for receiving a package
        double checkInterval;           //Time for cheking the chanel
//        double CCAInterval;

        double sendDelay;               //Time before sending a package
        double maximumBackOffTime;      //Maximum Time for turning off the radio
        double minimumBackOffTime;
        double resolutionBackOffTime;   //Number of back of times
        double sleepDuration;           //Time for turning off the radio
//        double interPeriodicWakeupDuration; //Time between periodic wake up spikes
        double periodicWakeupDuration; //the duration of lasting of the periodic wake up spike
        int consecutiveCCAAttempts;
        int boxmacSwitchingCount;
        int maxAllowedBroadcastMacSwitchingCount;

        double congestion;              //chanel free indicator
        bool killFlag;                  //Flag for destroyed SN
        bool offFlag;                   //Flag for deactivated SN
        bool isSink;                    //Flag for base station
        bool periodicWakeupInitiated;   //Flag for indicating the activation of periodic waking up (during the sleep state)
        bool packetReceivedDuringCCA;   //Flag for indicating that a packet has been received during the CCA state
        bool packetReceivedDuringINIT;  //Flag for indicating that a packet has been received during the INIT state
        bool isWakeupScheduled;         //Flag for indicating if the 'wakeup' event is scheduled and to be triggered in the future
        bool isPacketBroadcast;         //Flag for indicating if the packet that's been transmitted currently is a broadcast
        bool CCAAttempt1Clear;          //This flag indicates, that during the first attempt of CCA, the channel is free. The node is clear to go to the 2nd attempt of CCA after a period equivalent to 'waitInterval'.

        //------------Q learning related--------------------------------
        double ackReceptionTime;
        double lastPacketRetransmissionTime;
        int retransmissionCount;
        double listeningStartTime;
        double dataPacketReceptionTime;
        bool dataPacketReceived;
        double myParameterRiskiness;
        bool isDistributed;
        double cumulativeBackoffTime;
        double cumulativeTimeToGateway;
        double queuingTime;
        double transmissionDelay;   //the time delay in transmitting to the parent node (one way transmission)
        bool isCCASleep;
        bool fire;
        double lastPacketSendTime;  //the time the last packet has been send
        int lastMeasurementSendCount;    //number of measurement in the last packet sent

        //debug
        bool jj;
        bool ll;
        bool executeOnce;
        bool getSimulationOn;
        int ackCount;
        double globalBackoff;
        int sensorNumber;
        int node0State;
        double wakeUpTime;
        int CCAAttempt1ClearCount;
        //debug logs
        std::string cumTimeToGatewayRecords;

        //model_sim
        std::string modelSimTimeStartFileLocation;
        std::string modelSimTimeEndFileLocation;
        std::string modelSimRadioStateFileLocation;
        std::string stateLog;
        //--------------------------------------------------------------

        /** @brief The bitrate of transmission */
        double bitrate;
        /** @brief Internal function to change the color of the node */
        void changeDisplayState();
        /** @brief Internal function to send the first packet in the queue */
        void sendDataPacket();
        /** @brief Internal function to send an ACK */
        void sendMacAck();
        /** @brief Internal function to attach a signal to the packet */
        void attachSignal(Packet *packet, simtime_t_cref startTime);
    };
} // namespace inet
#endif /* BOXMAC_H_ */
