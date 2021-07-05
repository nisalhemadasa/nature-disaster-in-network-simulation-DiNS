#ifndef __INET_BOXMACPROTOCOLPRINTER_H
#define __INET_BOXMACPROTOCOLPRINTER_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/printer/ProtocolPrinter.h"

namespace inet {

class INET_API BOXMacProtocolPrinter : public ProtocolPrinter
{
  public:
    virtual void print(const Ptr<const Chunk>& chunk, const Protocol *protocol, const cMessagePrinter::Options *options, Context& context) const override;
};

} // namespace inet

#endif // __INET_BOXMACPROTOCOLPRINTER_H

