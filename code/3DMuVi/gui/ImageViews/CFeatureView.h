#ifndef CFEATUREVIEW_H
#define CFEATUREVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

class CFeatureView : public CImageView, public IDataView
{
  Q_OBJECT

public:
  //TODO: Include data packets
  //void applyData(CImageDataPacket* packet);
  //void applyData(CFeatureDataPacket* packet);
  void activate();

public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CFEATUREVIEW_H
