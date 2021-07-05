#include "StaticManualMobility.h"

namespace inet{
    Define_Module(StaticManualMobility);
    void StaticManualMobility::setInitialPosition(){
        lastPosition.x=par("initialX");
        lastPosition.y=par("initialY");
        lastPosition.z=par("initialZ");
    }
} // namespace inet

