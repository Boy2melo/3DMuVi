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
 * \author Nathanael Schneider
 *
 * Interface for DataPackets.
 */
class IDataPacket
{
private:
  QUuid mUuid;

public:
  /*!
  \brief Empty destructor. To be overriden by derived classes.
  */
  virtual ~IDataPacket() {}
  /*!
  \brief Creates the ID of this packet.
  */
  IDataPacket();

  /*!
  \brief Provide the ID of the data packet.
  \return ID of the data packet.
  */
  QString getId() const;

  /*!
  \brief Provide the name of the data type.
  \return Name of the data type.
  */
  virtual QString getDataType() const = 0;

  /*!
  \brief Apply this packet on the given data view.
  \param dataView The data view to apply this packet.
  */
  virtual void applyToDataview(IDataView* dataView) const = 0;

  /*!
  \brief Provides a stream provider.
  \return A stream provider in which this packet can be serialized.
  */
  virtual AStreamProvider* getStreamProvider() = 0;

  /*!
   * \brief Cleans up the last returned stream provider.
   */
  virtual void cleanUpStreamProvider() = 0;

  /*!
  \brief Serialize the data packet. To get a AStreamProvider use getStreamProvider(). After you got
  the AStreamProvider, you have to set the destination path.
  \param stream Provides a stream in which the data gets serialized.
  */
  virtual void serialize(AStreamProvider* stream) = 0;

  //TODO: Is AStreamProvider able to deserialize a packet?
  /*!
  \brief Deserialize the data packet.
  \param stream Provides a stream in which the data gets serialized.
  */
  virtual void deserialize(AStreamProvider* stream) = 0;
};

#endif // IDATAPACKET_H
