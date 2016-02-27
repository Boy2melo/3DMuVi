#ifndef IDATAPACKET_H
#define IDATAPACKET_H

#include <QString>
#include "io/AStreamProvider.h"
#include <QUuid>
#include <QDataStream>
#include <memory>

class IDataView;

/*!
 * \class IDataPacket
 * \brief The IDataPacket interface
 * \author ???
 *
 * Interface for DataPackets.
 */
class IDataPacket {
private:
    QUuid mUuid;

public:
    virtual ~IDataPacket() {}
    IDataPacket();

    /*!
    \brief Provide the id of the data packet.
    \return QString id of the data type.
    */
    QString getId() const;

    /*!
    \brief Provide the name of the data type.
    \return QString Name of the data type.
    */
    virtual QString getDataType() const = 0;

    /*!
    \brief ???
    \param *dataView ???
    */
    void ApplyToDataview(IDataView *dataView) const;

    /*!
    \brief Provides an AStreamProvider.
    \return AStreamProvider
    */
    virtual AStreamProvider* getStreamProvider() = 0;

    /*!
    \brief Serialize the data packet. To get a AStreamProvider use getStreamProvider(). After you got the AStreamProvider, you have to the destination path.
    \param stream provides a stream in which the data get serialized.
    */
    virtual void serialize(AStreamProvider* stream) = 0;

    /*!
    \brief Deserialize the data packet.
    \param stream provides a stream in which the data get serialized.
    */
    virtual void deserialize(AStreamProvider* stream) = 0;
};

#endif // IDATAPACKET_H
