#ifndef IDATAPACKET_H
#define IDATAPACKET_H

#include <QString>
#include "idataview.h"
#include <QUuid>

class IDataPacket {
private:
    QUuid mUuid;

public:
    virtual ~IDataPacket() {}
    IDataPacket();
    QString getId() const;
    virtual QString getDataType() const = 0;
    void ApplyToDataview(IDataView *dataView) const;
};

#endif // IDATAPACKET_H
