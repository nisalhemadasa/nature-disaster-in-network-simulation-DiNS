#ifndef __INET_STATICGRIDMOBILITY_H
#define __INET_STATICGRIDMOBILITY_H

#include "inet/sensornode/database/SensorDB.h"
#include "inet/mobility/static/StationaryMobility.h"

namespace inet {
    class INET_API StaticManualMobility : public StationaryMobility{
        protected:
            virtual void setInitialPosition() override;
        public:
            StaticManualMobility() {};
    };

}

#endif

