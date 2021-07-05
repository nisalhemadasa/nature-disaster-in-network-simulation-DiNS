#include "SensorUnit.h"
#include <math.h>
namespace inet {
    Define_Module(SensorUnit);
    void SensorUnit::checkPulse(){
        double t =simTime().dbl();
        double sample= floor(t/samplingTime);
        double tm=sample*samplingTime;
        double Tp=sensordbFuntions->getTemperature(int(sample)+1, index);
//        if (Tp>=maxTemperature){
        if (Tp>=maxTemperature){
//            //debug Nisal
//            char text[128];
//            double T;
//            double R;
//            T=getTemperature()+uniform(-maxTnoise,maxTnoise);
//            R=getHumidity()+uniform(-maxHnoise,maxHnoise);
//            meassureRiskIndex(T,R);
//            sprintf(text,"fire! tempapp: %f, fire: %f, humidity: %f, fireCount: %d, riskIndex: %f", Tp, T, R,fireCount, riskInd);
//            getSimulation()->getActiveEnvir()->alert(text);
            //debug end------------------------------------
            double Tm=sensordbFuntions->getTemperature(int(sample), index);
            double t=samplingTime/(Tp-Tm)*(maxTemperature-Tm)+tm;
            scheduleAt(t,killMsg);
        }else{
            scheduleAt(tm+samplingTime,checkMsg);
        }
    }
    double SensorUnit::getHumidity(){
        double t =simTime().dbl();
        double sample= floor(t/samplingTime);
        double tm=sample*samplingTime;
        double Rm=sensordbFuntions->getHumidity(int(sample), index);
        double Rp=sensordbFuntions->getHumidity(int(sample)+1, index);
        return (Rp-Rm)/samplingTime*(t-tm)+Rm;
    }
    double SensorUnit::getTemperature(){
        double t =simTime().dbl();
        double sample= floor(t/samplingTime);
        double tm=sample*samplingTime;
        double Tm=sensordbFuntions->getTemperature(int(sample), index);
        double Tp=sensordbFuntions->getTemperature(int(sample)+1, index);
        return (Tp-Tm)/samplingTime*(t-tm)+Tm;
    }
    MatrixXd SensorUnit::shiftedByRows(MatrixXd in,int positions){
        MatrixXd out(in.rows(),in.cols());
        for (int i=in.rows()-1;i>=positions;i--){
            out.row(i)=in.row(i-positions);
        }
        return out;
    }
    MatrixXd SensorUnit::shiftedByCols(MatrixXd in,int positions){
        MatrixXd out(in.rows(),in.cols());
        for (int i=in.cols()-1;i>=positions;i--){
            out.col(i)=in.col(i-positions);
        }
        return out;
    }
    void SensorUnit::meassureRiskIndex(double T, double R){
        double riskIndex;
        riskIndex = (R/20)+((27-T)/10);  //Angstroem Index
        riskInd = riskIndex;
//        if (riskIndex<0){     // commented by Nisal
//            riskIndex=0;
//        }
        routingFuntions->setRiskIndex(riskIndex);
    }
    MatrixXd SensorUnit::buildAfromStates(){
        MatrixXd Amatrix;
        Amatrix = MatrixXd::Zero(stateEquations,stateOrder*2);
        for(int i=0;i<=Amatrix.cols()-2;i+=2){
            Amatrix.block(0,i,Amatrix.rows(),2)=memoryStates.block(i/2,0,Amatrix.rows(),2);
        }
        return Amatrix;
    }
    void SensorUnit::sendStates(){
        double stateTime;
        double now;
        int delta;
        now=simTime().dbl();
        delta=double((now-lastTime)/meassureTime);
        if (delta>memoryStates.rows()){
            delta=memoryStates.rows();
        }
        for (int i=0;i<delta;i++){
            stateTime=simTime().dbl()-i*meassureTime;
            routingFuntions->addMyData(memoryStates(i,0), 0, stateTime);
            routingFuntions->addMyData(memoryStates(i,1), 1, stateTime);
        }
        lastTime=now;
    }
    void SensorUnit::periodicReadSensors(){
        double stateTime;
        double T;
        double R;
        stateTime=simTime().dbl();
        T=getTemperature()+uniform(-maxTnoise,maxTnoise);
        R=getHumidity()+uniform(-maxHnoise,maxHnoise);
        meassureRiskIndex(T,R);
        routingFuntions->addMyData(T, 0, stateTime);
        routingFuntions->addMyData(R, 1, stateTime);
    }
    void SensorUnit::eventReadSensors(){
        periodic = par("periodic"); //Periodic or event scheduling
        /////////////////////Prefilter/////////////////////////
        //Real meassurements
        bMeassures=shiftedByRows(bMeassures,1);
        bMeassures(0,0)=getTemperature()+uniform(-maxTnoise,maxTnoise);
        bMeassures(0,1)=getHumidity()+uniform(-maxHnoise,maxHnoise);
        //Adapt system
        xMeassures=AMeassures.fullPivHouseholderQr().solve(bMeassures);
        //Real states
        MatrixXd newStates=AMeassures.row(0)*xMeassures;
        //Shift past meassurements
        AMeassures=shiftedByCols(AMeassures,2);
        AMeassures.block(0,0,AMeassures.rows(),2)=bMeassures;
        //Calculate risk index
        meassureRiskIndex(newStates(0,0),newStates(0,1));
        /////////////////////Real states/////////////////////////
        memoryStates=shiftedByRows(memoryStates,1);
        memoryStates.block(0,0,1,2)=newStates;
        /////////////////////Predict/////////////////////////
        //Predicted states
        MatrixXd predictedStates=AStates.row(0)*xStates;
        double temperatureError;
        temperatureError = abs(predictedStates(0,0)-newStates(0,0));
        double humidityError;
        humidityError = abs(predictedStates(0,1)-newStates(0,1));
        if(temperatureError>temperatureThreshold || humidityError>humidityThreshold){
            AStates=buildAfromStates();
            bStates=memoryStates.block(0,0,bStates.rows(),2);   //Change for real states
            xStates=AStates.fullPivHouseholderQr().solve(bStates); //Adapt predictor
            sendStates(); //Send states

            lasm++;
            EV<<lasm<<"adaptation \n";
        }else{
            bStates=shiftedByRows(bStates,1);
            bStates.block(0,0,1,2)=predictedStates; //Add predicted states
        }
        //Shift past states
        AStates=shiftedByCols(AStates,2);
        AStates.block(0,0,AStates.rows(),2)=bStates;
    }

    void SensorUnit::initialize(){
        lasm=0;

        cModule *node;
        node = getParentModule();
        cModule *routing;
        routing = node->getSubmodule("sensorRouting", 0);
        routingFuntions = check_and_cast<SensorRouting *>(routing);
        cModule *wlan;
        wlan=node->getSubmodule("wlan", 0);
        cModule *mac;
        mac=wlan->getSubmodule("mac");
        macFuntions = check_and_cast<BOXMac *>(mac);
        isSink = par("isSink");
        routingFuntions->setSinkRouting(isSink);
        if (!isSink){
            cModule *network;
            network = node->getParentModule();
            cModule *sensordb;
            sensordb=network->getSubmodule("sensordb");
            sensordbFuntions=check_and_cast<SensorDB *>(sensordb);
            index = node->getIndex();
            samplingTime = par("samplingTime");
            numberSamples = par("numberSamples");
            maxTemperature = par("maxTemperature");
            meassureTime = par("meassureTime");
            maxStartTime = par("maxStartTime");
            readOrder = par("readOrder");
            readEquations = int(par("readMemory"))-readOrder+1; //Number of equations for Minimum mean square
            stateOrder = par("stateOrder");
            stateEquations = int(par("stateMemory"))-stateOrder+1; //Number of equations for Minimum mean square
            maxTnoise = par("maxTnoise");
            maxHnoise = par("maxHnoise");
            temperatureThreshold = par("temperatureThreshold");
            humidityThreshold = par("humidityThreshold");
            checkMsg = new cMessage("checkEvent");
            checkMsg->setKind(CHECKEVENT);
            killMsg = new cMessage("killEvent");
            killMsg->setKind(KILLEVENT);
            readMsg = new cMessage("readEvent");
            readMsg->setKind(READEVENT);
            AMeassures=MatrixXd::Zero(readEquations,readOrder*2); //past readings
            AMeassures(0,0)=1; //Initialize the matrix with ones to avoid zeros at the beginning
            AMeassures(0,1)=1;
            bMeassures=MatrixXd::Zero(readEquations,2); //new states
            xMeassures=MatrixXd::Zero(stateOrder*2,2); //plant
            AStates=MatrixXd::Zero(stateEquations,stateOrder*2); //past states
            bStates=MatrixXd::Zero(stateEquations,2); //new states
            xStates=MatrixXd::Zero(stateOrder*2,2); //plant
            memoryStates=MatrixXd::Zero(stateEquations+readOrder-1,2); //realstates
            double randomStart=uniform(0,maxStartTime);
            lastTime=randomStart;
            scheduleAt(randomStart+meassureTime,readMsg);
            scheduleAt(samplingTime,checkMsg);
        }
    }

    void SensorUnit::handleMessage(cMessage *msg){
        if (msg->getKind()==CHECKEVENT){
            checkPulse();
        }else if (msg->getKind()==KILLEVENT){
            macFuntions->setDead();
            cancelAndDelete(readMsg);
        }else if (msg->getKind()==READEVENT){
            if (periodic){
                periodicReadSensors();
            }else{
                eventReadSensors();
            }
            scheduleAt(simTime()+meassureTime,readMsg);
        }
    }

} //namespace
