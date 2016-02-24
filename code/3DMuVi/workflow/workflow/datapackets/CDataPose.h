#ifndef CDATAPOSE_H
#define CDATAPOSE_H

#include <tuple>

#include "workflow/workflow/idatapacket.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "workflow/workflow/ccontextdatastore.h"
#include "io/AStreamProvider.h"
#include "io/CSFStreamProvider.h"

using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;
using FeatureMap = std::vector<std::tuple<uint32_t, float, float, float, float>>;

/*!
 * \class CDataPose
 * \brief The CDataPose class
 * \author Laurenz Thiel
 *
 * Erster Entwurf - Komplett ungetestet.
 */
class CDataPose : public IDataPacket {
public:
    CDataPose();
    ~CDataPose();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void serialize(AStreamProvider* stream) override;

    FeatureMatch const & getFeatureMatch() const;
    void setFeatureMatch(FeatureMatch && feature);

    FeatureMap const & getFeatureMap() const;
    void setFeatureMap(FeatureMap && map);

    std::vector<SPose> const & getPose() const;
    void setPose(std::vector<SPose> && poses);

private:
    AStreamProvider* streamProvider;
    FeatureMatch featureMatch;
    FeatureMap featureMap;
    std::vector<SPose> pose;
};

#endif // CDATAPOSE_H
