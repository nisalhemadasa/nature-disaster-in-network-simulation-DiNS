package inet.sensornode.mobility.staticmanual;

import inet.mobility.base.MobilityBase;

simple StaticManualMobility extends MobilityBase
{
    parameters:
		double initialX @unit(m) = default(0m);
        double initialY @unit(m) = default(0m);
        double initialZ @unit(m) = default(0m);
        @class(StaticManualMobility);
}

