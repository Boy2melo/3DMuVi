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
  void applyData(std::shared_ptr<CInputDataSet const> packet) override;
  void applyData(std::shared_ptr<CDataFeature const> packet) override;
  void activate() override;
  void clearData() override;

private:

  std::vector<uint32_t> mDataID;
  std::vector<QImage*> mImageList;
  std::shared_ptr<CInputDataSet const> appliedInputData = nullptr;
  std::shared_ptr<CDataFeature const> appliedFeatureData = nullptr;


  void updateView();


public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CFEATUREVIEW_H
