#ifndef TDATAPACKET_H
#define TDATAPACKET_H

#include "workflow/workflow/idatapacket.h"

template<typename T>
class TDataPacket : IDataPacket
{
public:
    TDataPacket();
    T* getData() const;
};

#endif // TDATAPACKET_H
