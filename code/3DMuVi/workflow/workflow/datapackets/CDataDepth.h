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

/*!
 * \class CDataDepth
 * \brief The CDataDepth class
 * \author Laurenz Thiel
 *
 * Erster Entwurf - Komplett ungetestet.
 */
class CDataDepth : public IDataPacket {
public:
    CDataDepth();
    ~CDataDepth();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void serialize(AStreamProvider* stream) override;

    std::shared_ptr<std::vector<QImage>> getDepthMap() const;
    void setDepthMap(std::shared_ptr<std::vector<QImage>> depthMaps);
private:
    AStreamProvider* streamProvider;
    std::shared_ptr<std::vector<QImage>> depthMap;
};

#endif // CDATADEPTH_H
