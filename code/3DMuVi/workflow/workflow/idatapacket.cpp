#include "idatapacket.h"
#include "idataview.h"

IDataPacket::~IDataPacket() {}

IDataPacket::IDataPacket()
{
  mUuid = QUuid::createUuid();
}

QString IDataPacket::getId() const
{
  return mUuid.toString();
}
