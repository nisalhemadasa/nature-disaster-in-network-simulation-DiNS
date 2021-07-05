#include "BOXMacProtocolDissector.h"
#include "inet/common/ProtocolGroup.h"
#include "inet/common/packet/dissector/ProtocolDissectorRegistry.h"
#include "BOXMacHeader_m.h"


namespace inet {

Register_Protocol_Dissector(&Protocol::boxmac, BOXMacProtocolDissector);

void BOXMacProtocolDissector::dissect(Packet *packet, ICallback& callback) const
{
    auto header = packet->popAtFront<BOXMacHeader>();
    callback.startProtocolDataUnit(&Protocol::boxmac);
    callback.visitChunk(header, &Protocol::boxmac);
    if (header->getType() == BOXMAC_DATA) {
        auto payloadProtocol = ProtocolGroup::ethertype.getProtocol(header->getNetworkProtocol());
        callback.dissectPacket(packet, payloadProtocol);
    }
    ASSERT(packet->getDataLength() == B(0));
    callback.endProtocolDataUnit(&Protocol::boxmac);
}

} // namespace inet

