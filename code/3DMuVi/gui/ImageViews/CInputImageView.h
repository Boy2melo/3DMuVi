#ifndef CINPUTIMAGEVIEW_H
#define CINPUTIMAGEVIEW_H

#include <workflow/workflow/idataview.h>

#include "CImageView.h"

class CInputDataSet;

class CInputImageView : public CImageView, public IDataView
{
    Q_OBJECT

public:
  explicit CInputImageView();
  void applyData(const CInputDataSet* packet) override;
  void activate() override;

private:
  const CInputDataSet* appliedData = nullptr;
  std::vector<uint32_t> mDataID;
  std::vector<QImage*> mImageList;

  void updateView();

public slots:
  void onImagesSelected(std::vector<uint32_t>& images);

signals:
  void relevantImagesChanged(std::vector<uint32_t>& images);
};

#endif // CINPUTIMAGEVIEW_H
