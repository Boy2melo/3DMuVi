#ifndef CINPUTIMAGEVIEW_H
#define CINPUTIMAGEVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

class CInputImageView : public CImageView, public IDataView
{
  Q_OBJECT

public:
  explicit CInputImageView();
  //TODO: Include data packets
  //void applyData(CImageDataPacket* packet);
  void activate();

public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CINPUTIMAGEVIEW_H
