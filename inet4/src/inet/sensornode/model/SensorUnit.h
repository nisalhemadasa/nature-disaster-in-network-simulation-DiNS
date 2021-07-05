#ifndef __INET4_SENSORUNIT_H_
#define __INET4_SENSORUNIT_H_

#include <omnetpp.h>

#include "inet/sensornode/database/SensorDB.h"
#include "inet/sensornode/linklayer/boxmac/BOXMac.h"
#include "inet/sensornode/routing/SensorRouting.h"
#include "inet/libraries/Eigen/Dense"

using namespace Eigen;
using namespace omnetpp;

#define CHECKEVENT          0
#define KILLEVENT           1
#define READEVENT           2

namespace inet {
    class SensorUnit : public cSimpleModule{
        protected:
            void checkPulse();
            double getHumidity();
            double getTemperature();
            virtual void initialize();
            virtual void handleMessage(cMessage *msg);
            virtual MatrixXd shiftedByCols(MatrixXd in,int positions);
            MatrixXd shiftedByRows(MatrixXd in,int positions);
            virtual void meassureRiskIndex(double T, double R);
            virtual void eventReadSensors();
            virtual void periodicReadSensors();
            virtual MatrixXd buildAfromStates();
            virtual void sendStates();
            double samplingTime;
            double meassureTime;
            double maxStartTime;
            int numberSamples;
            int index;

            int readOrder;
            int readEquations;
            int stateOrder;
            int stateEquations;

            double maxTnoise;
            double maxHnoise;

            double temperatureThreshold;
            double humidityThreshold;

            bool isSink;
            bool periodic;
            double lastTime; //Last time the meassurements where added to the queue

            MatrixXd AMeassures;
            MatrixXd bMeassures;
            MatrixXd xMeassures;
            MatrixXd AStates;
            MatrixXd bStates;
            MatrixXd xStates;
            MatrixXd memoryStates;

            SensorDB *sensordbFuntions;
            double maxTemperature;
            cMessage *checkMsg;
            cMessage *killMsg;
            cMessage *readMsg;
            BOXMac *macFuntions;

            SensorRouting *routingFuntions;

            int lasm;

            // debug Nisal
            int fireCount;
            double riskInd;

};

} //namespace

#endif
