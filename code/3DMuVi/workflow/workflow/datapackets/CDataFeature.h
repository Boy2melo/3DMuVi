#ifndef CDATAFEATURE_H
#define CDATAFEATURE_H
#include "workflow/workflow/idatapacket.h"
#include "io/AStreamProvider.h"
#include "io/CSFStreamProvider.h"

using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;

/*!
 * \class CDataFeature
 * \brief The CDataFeature class
 * \author Laurenz Thiel
 *
 * Erster Entwurf - Komplett ungetestet.
 */
class CDataFeature : public IDataPacket {
public:
    CDataFeature();
    ~CDataFeature();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void serialize(AStreamProvider* stream) override;
    void setFeatureMatch(FeatureMatch && match);
    FeatureMatch const & getFeatureMatch() const;
private:
    FeatureMatch featureMatchData;
    AStreamProvider* streamProvider;
};

#endif // CDATAFEATURE_H
