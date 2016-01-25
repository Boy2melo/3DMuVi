#ifndef CDEPTHMAPVIEW_H
#define CDEPTHMAPVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

class CDepthMapView : public CImageView, public IDataView {
  Q_OBJECT

public:
  //TODO: Include data packets
  //void applyData(CDepthMapDataPacket* packet);
  void activate();

public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CDEPTHMAPVIEW_H
