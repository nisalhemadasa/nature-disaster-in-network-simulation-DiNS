package inet.sensornode.routing;

simple SensorRouting
{
    parameters:
        string riskFunctionWeights = default("1.0 1.0 1.0");
        string checkIntervalsSet = default("0.04 0.04 0.04");
        string slotDurationsSet = default("0.45 0.45 0.45");
        string iterationCount = default("1");
        string memoryClassMargins = default("1 2 4 8 16 32 64 128 256");
        string congestionClassMargins = default("1  2 3 4 100 200 400 800");
        string batteryClassMargins = default("2 3 5 7 10 14 20 28 40 58 82 118 168 240 343 490 700");
        string classMargins = default("2 2.5 3 4");
        double initialBattery @unit(J) = default(1J);
        double minBattery @unit(J) = default(0.1J);
        double thresholdBattery @unit(J) = default(0.001J);
        double thresholdRiskIndex = default(1);
        double minTrikleTime @unit(s) = default(1s);
        double updateTime @unit(s) = default(1s);
        double maxTrikleTime @unit(s) = default(128s);
        double maxNeighborTime @unit(s) = default(512s);
        int maxNumberHops = default(25);
        int maxDataSize @unit(b) = default(1016b);
        int meassurementSize @unit(b) = default(64b);
        int parametersSize @unit(b) = default(64b);
        int headerLength @unit(b) = default(64b);
        int maxpenalties = default(5);
        double fireThreshold = default(2.5);

    gates:
        input macPackageIn @directIn;
}
