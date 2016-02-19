#ifndef CDATADEPTH_H
#define CDATADEPTH_H

#include <tuple>
#include <QImage>
#include <QBuffer>

#include "workflow/workflow/idatapacket.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "workflow/workflow/ccontextdatastore.h"
#include "io/AStreamProvider.h"
#include "io/CMFStreamProvider.h"

using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;
using FeatureMap = std::vector<std::tuple<uint32_t, float, float, float, float>>;

/*!
 * \class CDataDepth
 * \brief The CDataDepth class
 * \author Laurenz Thiel
 *
 * Erster Entwurf - Komplett ungetestet.
 */
class CDataDepth : public IDataPacket
{
public:
    CDataDepth();
    ~CDataDepth();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider();
    void serialize(AStreamProvider* stream);

    FeatureMatch const & getFeatureMatch();
    void setFeatureMatch(FeatureMatch && feature);

    FeatureMap const & getFeatureMap();
    void setFeatureMap(FeatureMap && map);

    std::vector<SPose> const & getPose();
    void setPose(std::vector<SPose> && poses);

    std::vector<QImage> const & getDepthMap();
    void setDepthMap(std::vector<QImage> && depthMaps);
private:
    AStreamProvider* streamProvider;
    FeatureMatch featureMatch;
    FeatureMap featureMap;
    std::vector<SPose> pose;
    std::vector<QImage> depthMap;
};

#endif // CDATADEPTH_H
