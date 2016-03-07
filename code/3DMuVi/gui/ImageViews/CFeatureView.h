#ifndef CFEATUREVIEW_H
#define CFEATUREVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

using ImageFeature = std::tuple<uint32_t, QVector2D>;

class CInputDataSet;
class CDataFeature;

class CFeatureView : public CImageView, public IDataView
{
  Q_OBJECT

public:
  void applyData(const CInputDataSet* packet) override;
  void applyData(const CDataFeature* packet) override;
  void activate();

private:

  std::vector<uint32_t> mDataID;
  std::vector<QImage*> mImageList;
  const CInputDataSet* appliedInputData;
  const CDataFeature* appliedFeatureData;


  void updateView();


public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CFEATUREVIEW_H
