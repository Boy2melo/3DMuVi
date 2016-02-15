#ifndef IDATAPACKET_H
#define IDATAPACKET_H

#include <QString>
#include "idataview.h"
#include <QUuid>
#include <QDataStream>

class IDataPacket {
private:
    QUuid mUuid;

public:
    virtual ~IDataPacket() {}
    IDataPacket();
    QString getId() const;
    virtual QString getDataType() const = 0;
    void ApplyToDataview(IDataView *dataView) const;
    virtual void serialize(QDataStream* stream) = 0;
    virtual void deserialize(QDataStream* stream) = 0;
};

#endif // IDATAPACKET_H
