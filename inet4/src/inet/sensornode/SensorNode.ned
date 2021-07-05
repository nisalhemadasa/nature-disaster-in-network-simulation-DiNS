package inet.sensornode;

import inet.node.base.LinkLayerNodeBase;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarTransmitter;
import inet.sensornode.record.Record;
import inet.sensornode.routing.SensorRouting;
import inet.sensornode.model.SensorUnit;

module SensorNode extends LinkLayerNodeBase
{
    parameters:
        @display("i=misc/sensor2;bgb=1302.5249,669.375");
        @figure[submodules];
        numWlanInterfaces = default(1);
        energyStorage.typename = default("IdealEpEnergyStorage");
        wlan[*].typename = default("Ieee802154NarrowbandInterface");
        wlan[*].radio.energyConsumer.typename = default("SensorStateBasedEpEnergyConsumer");
        wlan[*].mac.typename = default("XMac");
    submodules:
        sensorUnit: SensorUnit {
            @display("p=511.87497,159.075");
        }
        sensorRouting: SensorRouting {
            @display("p=770.175,159.075");
        }
        record: Record {
            @display("p=1185.975,72.45");
        }
}
