#ifndef __INET_STATICGRIDMOBILITY_H
#define __INET_STATICGRIDMOBILITY_H

#include "../../database/SensorDB.h"
#include "inet/mobility/static/StationaryMobility.h"

namespace inet {
    class INET_API StaticFileMobility : public StationaryMobility{
        protected:
            virtual void setInitialPosition() override;
        public:
            StaticFileMobility() {};
    };

}

#endif

