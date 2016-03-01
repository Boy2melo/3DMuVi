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
  //TODO: Include data packets
  void applyData(CInputDataSet* packet);
  void applyData(CDataFeature* packet);
  void activate();

private:

  std::vector<uint32_t> mDataID;
  CInputDataSet* appliedInputData;
  CDataFeature* appliedFeatureData;


  void updateView();


public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CFEATUREVIEW_H
