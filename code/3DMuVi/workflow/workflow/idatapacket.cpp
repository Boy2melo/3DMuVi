#include "idatapacket.h"
#include "idataview.h"

IDataPacket::IDataPacket() {
    mUuid = QUuid::createUuid();
}

QString IDataPacket::getId() const {
    return mUuid.toString();
}

void IDataPacket::ApplyToDataview(IDataView* dataView) const {
    dataView->applyData(this);
}
