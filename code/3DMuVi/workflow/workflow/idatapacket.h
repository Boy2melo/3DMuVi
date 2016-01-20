#ifndef IDATAPACKET_H
#define IDATAPACKET_H

#include <QString>

class IDataPacket
{
public:
    IDataPacket();
    QString getId() const;
    QString getDataType() const;
    void* getData();
};

#endif // IDATAPACKET_H
