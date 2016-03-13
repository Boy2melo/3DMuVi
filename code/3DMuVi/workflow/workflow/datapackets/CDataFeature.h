#ifndef CDATAFEATURE_H
#define CDATAFEATURE_H

#include "macros.h"
#include "workflow/workflow/idatapacket.h"
#include "io/AStreamProvider.h"
#include "io/CSFStreamProvider.h"

using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;

/*!
 * \class CDataFeature
 * \brief The CDataFeature class
 * \author Laurenz Thiel
 *
 * Datapacket which contains the FeatureMatch data.
 * It can be serialized into a stream and deserialized from a stream.
 */
class EXPORTED CDataFeature : public IDataPacket {
public:
    CDataFeature();
    ~CDataFeature();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void cleanUpStreamProvider() override;
    void serialize(AStreamProvider* stream) override;
    void deserialize(AStreamProvider* stream) override;

    /*!
    \brief Sets the FeatureMatch
    \param match A shared pointer which point to a FeatureMatch.
    */
    void setFeatureMatch(std::shared_ptr<FeatureMatch> match);
    /*!
    \brief Get the FeatureMatch
    \return std::shared_ptr<FeatureMatch> Returns a shared pointer to the FeatureMatch.
    */
    std::shared_ptr<FeatureMatch> getFeatureMatch() const;
private:
    std::shared_ptr<FeatureMatch> featureMatchData;
    AStreamProvider* streamProvider;
};

#endif // CDATAFEATURE_H
