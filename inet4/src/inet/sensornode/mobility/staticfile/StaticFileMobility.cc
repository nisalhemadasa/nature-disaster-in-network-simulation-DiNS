#include "../../mobility/staticfile/StaticFileMobility.h"

namespace inet{
    Define_Module(StaticFileMobility);
    void StaticFileMobility::setInitialPosition(){
        cModule *node;
        node = getParentModule();
        cModule *network;
        network = node->getParentModule();
        int index;
        cModule *sensordb;
        sensordb=network->getSubmodule("sensordb");
        SensorDB *sensordbFuntions;
        sensordbFuntions=check_and_cast<SensorDB *>(sensordb);
        index = node->getIndex();
        lastPosition=sensordbFuntions->getPosition(index);
    }
} // namespace inet

