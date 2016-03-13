#ifndef CDEPTHMAPVIEW_H
#define CDEPTHMAPVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

/*!
 * \class CDepthMapView
 * \brief CDepthMapView class
 * \author Grigori Schapoval
 *
 */

class CDataDeptht;

class CDepthMapView : public CImageView, public IDataView {
  Q_OBJECT

public:
  void applyData(const CDataDepth* packet) override;
  void activate() override;
  void clearData() override;

private:
  const CDataDepth* appliedData = nullptr;
  std::vector<uint32_t> mDataID;
  std::vector<QImage*> mImageList;

  void updateView();

public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CDEPTHMAPVIEW_H
