import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
namespace inet;

class ParameterChunk extends FieldsChunk{
    bool Fire;
    double Battery;
    double Congestion;
    double RiskIndex;
    int NumberHops;
    int QueSize;
}
