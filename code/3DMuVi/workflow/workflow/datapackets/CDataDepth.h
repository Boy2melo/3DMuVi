#ifndef CDATADEPTH_H
#define CDATADEPTH_H

#include <tuple>
#include <QImage>
#include <QBuffer>

#include "macros.h"
#include "workflow/workflow/idatapacket.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "workflow/workflow/ccontextdatastore.h"
#include "io/AStreamProvider.h"
#include "io/CMFStreamProvider.h"

/*!
 * \class CDataDepth
 * \brief The CDataDepth class
 * \author Laurenz Thiel
 *
 * Datapacket which contains the depth maps.
 * It can be serialized into a stream and deserialized from a stream.
 */
class EXPORTED CDataDepth : public IDataPacket,
                            public std::enable_shared_from_this<CDataDepth> {
public:
    CDataDepth();
    ~CDataDepth();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void cleanUpStreamProvider() override;
    void serialize(AStreamProvider* stream) override;
    void deserialize(AStreamProvider *stream) override;

    /*!
    \brief Get the DepthMap data.
    \return std::shared_ptr<std::vector<QImage>> Returns a shared pointer to a std::vector with QImages.
    */
    std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>> getDepthMap() const;
    /*!
    \brief Sets the DepthMap data.
    \param depthMaps A shared pointer which point to a std::vector with QImages.
    */
    void setDepthMap(std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>> depthMaps);

    void applyToDataview(IDataView* dataView) const override;
private:
    AStreamProvider* streamProvider;
    std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>> depthMap;
};

#endif // CDATADEPTH_H
