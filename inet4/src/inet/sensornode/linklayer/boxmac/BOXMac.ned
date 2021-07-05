package inet.sensornode.linklayer.boxmac;
//package inet.linklayer.ieee80211.mac;

import inet.linklayer.base.MacProtocolBase;
import inet.linklayer.contract.IMacProtocol;
import inet.linklayer.ieee80211.mac.Tx;
import inet.linklayer.ieee80211.mac.coordinationfunction.Dcf;
import inet.linklayer.ieee80211.mac.originator.RtsPolicy;
//import inet.linklayer.contract.IMacProtocol;


simple BOXMac extends MacProtocolBase like IMacProtocol
{
    parameters:
        @class(BOXMac);
        @signal[packetDropped](type=Packet);
        double periodicWakeupDuration @unit("s") = default(0.01s);
        double interPeriodicWakeupDuration @unit("s") = default(0.02s);
        double CCAInterval @unit("s") = default(0.0001s);
        string address = default("auto"); 						// MAC address as hex string (12 hex digits)
        double slotDuration @unit(s) = default(0.1s);    		//Time for transmiting data
        double waitInterval @unit(s) = default(0.017s);      	//Time for receiving ack
        double checkInterval @unit(s) = default(0.017s);   		//Time for cheking the chanel

//        double sendDelay @unit(s) = default(0.000192s);      	//Time before sending a package
        double sendDelay @unit(s) = default(0.0s);      	//Time before sending a package
        double maximumBackOffTime @unit(s) = default(0.00256s);	//Time for turning off the radio
        double resolutionBackOffTime = default(5);  			//Time for turning off the radio
        double sleepDuration @unit(s) = default(0.05s);  		//Time for turning off the radio       
        int AckHeaderLength @unit(b) = default(40 b);			//Size of ack message
        // size of the MAC queue (maximum number of packets in Tx buffer)
        int queueLength = default(20);
        // bit rate 
        double bitrate @unit(bps) = default(19200 bps);
        // length of MAC header
        int mtu @unit(B) = default(0 B);
        string radioModule = default("^.radio");   // The path to the Radio module  //FIXME remove default value

        bool isSink = default(false);
}

//module BOXMac extends MacProtocolBase like IMacProtocol
//{
//    parameters:
//        @class(BOXMac);
//        @signal[packetDropped](type=Packet);
//        double periodicWakeupDuration @unit("s") = default(0.01s);
//        double interPeriodicWakeupDuration @unit("s") = default(0.02s);
//        double CCAInterval @unit("s") = default(0.0001s);
//        string address = default("auto"); 						// MAC address as hex string (12 hex digits)
//        double slotDuration @unit(s) = default(0.1s);    		//Time for transmiting data
//        double waitInterval @unit(s) = default(0.017s);      	//Time for receiving ack
//        double checkInterval @unit(s) = default(0.017s);   		//Time for cheking the chanel
//
//        double sendDelay @unit(s) = default(0.000192s);      	//Time before sending a package
//        double maximumBackOffTime @unit(s) = default(0.00256s);	//Time for turning off the radio
//        double resolutionBackOffTime = default(5);  			//Time for turning off the radio
//        double sleepDuration @unit(s) = default(0.05s);  		//Time for turning off the radio       
//        int AckHeaderLength @unit(b) = default(40 b);			//Size of ack message
//        // size of the MAC queue (maximum number of packets in Tx buffer)
//        int queueLength = default(20);
//        // bit rate 
//        double bitrate @unit(bps) = default(19200 bps);
//        // length of MAC header
//        int mtu @unit(B) = default(0 B);
//        string radioModule = default("^.radio");   // The path to the Radio module  //FIXME remove default value
//
//        bool isSink = default(false);
//    submodules:
//        dcf: Dcf {
//            @display("p=98,76");
//        }
//}


