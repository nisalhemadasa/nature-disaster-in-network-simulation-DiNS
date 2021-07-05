import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.linklayer.common.MacAddress;

namespace inet;
class DataChunk extends FieldsChunk{
    MacAddress MacName[];
    int Type[];
    double Time[];
    double value[];
}
