import inet.common.INETDefs;
import inet.linklayer.base.MacHeaderBase;

namespace inet;

enum BOXMacTypes {
    // packet types
    BOXMAC_FAIL = 191;
    BOXMAC_DATA = 192;
    BOXMAC_ACK = 193;
    BOXMAC_SUCCESS = 194;
    // self message types
    BOXMAC_START_BOXMAC = 196;
    BOXMAC_WAKE_UP = 197;
    BOXMAC_SEND_ACK = 198;
    BOXMAC_LISTEN_CHANNEL_TIMEOUT = 199;
    BOXMAC_ACK_TX_OVER = 200;
    BOXMAC_STOP_PREAMBLES = 202;
    BOXMAC_SWITCH_PREAMBLE_PHASE = 205;
    BOXMAC_DELAY_FOR_ACK_WITHIN_REMOTE_RX = 206;
    BOXMAC_SWITCHING_FINISHED = 207;
    BOXMAC_RETURN_DELAY = 2018;
    //Q learning related; inter-modular message types
    BOXMAC_LISTENING_FAILED = 2019;
    BOXMAC_PERIODIC_WAKEUP = 2020;
};

//
// Frame for ~BOXMac. Frame size is configurable
// in the MAC layer. 
//
class BOXMacHeader extends MacHeaderBase
{
    BOXMacTypes type;
    long sequenceId; // Sequence Number to detect duplicate messages
}

