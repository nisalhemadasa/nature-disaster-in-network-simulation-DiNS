package inet.sensornode.model;

simple SensorUnit{
    parameters:
        double maxTemperature = default(150);
        int numberSamples = default(1512);
        double samplingTime @unit(s) = default(1s);
        double meassureTime @unit(s) = default(1s);
        double maxStartTime @unit(s) = default(1s);
        
        int readOrder = default(9);
        int readMemory = default(62);
        int stateOrder = default(9);
        int stateMemory = default(26);    
        
        double maxTnoise = default(1);
        double maxHnoise = default(1);
        
        double temperatureThreshold = default(5);
        double humidityThreshold = default(5);
        
        bool isSink = default(false);
        
        bool periodic = default(true);
                
        @class(SensorUnit);
}
