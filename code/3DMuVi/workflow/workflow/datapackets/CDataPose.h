#ifndef CDATAPOSE_H
#define CDATAPOSE_H

#include <tuple>

#include "macros.h"
#include "workflow/workflow/idatapacket.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "workflow/workflow/ccontextdatastore.h"
#include "io/AStreamProvider.h"
#include "io/CSFStreamProvider.h"

using FeatureMap = std::vector<std::tuple<uint32_t, float, float, float, float>>;

/*!
 * \class CDataPose
 * \brief The CDataPose class
 * \author Laurenz Thiel
 *
 * Datapacket which contains the FeatureMap and Pose data.
 * It can be serialized into a stream and deserialized from a stream.
 */
class EXPORTED CDataPose : public IDataPacket,
                           public std::enable_shared_from_this<CDataPose> {
public:
    QString getDataType() const override;
    std::unique_ptr<AStreamProvider> getStreamProvider() override;
    void serialize(AStreamProvider* stream) override;
    void deserialize(AStreamProvider *stream) override;

    /*!
    \brief Get the FeatureMap
    \return std::shared_ptr<FeatureMap> Returns a shared pointer to the FeatureMap.
    */
    std::shared_ptr<FeatureMap> getFeatureMap() const;
    /*!
    \brief Sets the FeatureMap
    \param map A shared pointer which point to a FeatureMap.
    */
    void setFeatureMap(std::shared_ptr<FeatureMap>  map);

    /*!
    \brief Get the SPose data.
    \return std::shared_ptr<FeatureMap> Returns a shared pointer to a std::vector with SPoses.
    */
    std::shared_ptr<std::vector<SPose>> getPose() const;
    /*!
    \brief Sets the SPose data.
    \param map A shared pointer which point to a std::vector with SPoses.
    */
    void setPose(std::shared_ptr<std::vector<SPose>> poses);

    void applyToDataview(IDataView* dataView) const override;

private:
    std::shared_ptr<FeatureMap> featureMap;
    std::shared_ptr<std::vector<SPose>> pose;
};

#endif // CDATAPOSE_H
