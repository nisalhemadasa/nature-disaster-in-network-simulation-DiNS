#include "BOXMacProtocolPrinter.h"

#include "inet/common/packet/printer/PacketPrinter.h"
#include "inet/common/packet/printer/ProtocolPrinterRegistry.h"
#include "BOXMacHeader_m.h"

namespace inet {

Register_Protocol_Printer(&Protocol::boxmac, BOXMacProtocolPrinter);

void BOXMacProtocolPrinter::print(const Ptr<const Chunk>& chunk, const Protocol *protocol, const cMessagePrinter::Options *options, Context& context) const
{
    if (auto header = dynamicPtrCast<const BOXMacHeader>(chunk)) {
        context.sourceColumn << header->getSrcAddr();
        context.destinationColumn << header->getDestAddr();
        context.infoColumn << "BOXMAC type:" << header->getType() << " " << header;        //TODO
    }
    else
        context.infoColumn << "(BOXMAC) " << chunk;
}

} // namespace inet

