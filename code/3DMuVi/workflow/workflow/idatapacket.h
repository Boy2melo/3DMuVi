#ifndef IDATAPACKET_H
#define IDATAPACKET_H

#include <QString>
#include "idataview.h"
#include "io/AStreamProvider.h"
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
    virtual AStreamProvider* getStreamProvider() = 0;
    virtual void serialize(AStreamProvider* stream) = 0;
    virtual void deserialize(AStreamProvider* stream) = 0;
};

#endif // IDATAPACKET_H
