#ifndef __INET_BOXMACDISSECTOR_H
#define __INET_BOXMACDISSECTOR_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/dissector/ProtocolDissector.h"

namespace inet {

class INET_API BOXMacProtocolDissector : public ProtocolDissector
{
  public:
    virtual void dissect(Packet *packet, ICallback& callback) const override;
};

} // namespace inet

#endif // __INET_BOXMACDISSECTOR_H

