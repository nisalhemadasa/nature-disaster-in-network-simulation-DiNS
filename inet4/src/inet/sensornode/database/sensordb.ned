package inet.sensornode.database;

simple SensorDB
{
    parameters:
        string csvtemperature = default("");
        string csvhumidity = default("");
        string csvposition = default("");
        @display("i=abstract/db");
}
