package inet.sensornode.record;


simple Record
{
    parameters:
        string RecAddress = default("/home/leonardo/omnetpp-5.4.1/workspace/inet4/examples/sensornetwork/recordings");
        int MaxColumns = default(1024);
        @display("i=device/printer");
}
