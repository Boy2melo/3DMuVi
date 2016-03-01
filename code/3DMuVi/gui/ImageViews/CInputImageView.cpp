#include "CInputImageView.h"

#include "io/CInputDataSet.h"

//============================================================
/*!
@param packet
*/
//============================================================
void CInputImageView::applyData(CInputDataSet* packet)
{
    appliedData = packet;

    updateView();
}

//============================================================
/*!
*/
//============================================================
CInputImageView::CInputImageView()
{

}

void CInputImageView::activate()
{

  std::vector<uint32_t> images;
  for(std::tuple<uint32_t, QImage, CImagePreviewItem> i : *appliedData->getInputImages())
  {
    images.push_back(std::get<0>(i));
  }
  emit relevantImagesChanged(images);
}

//============================================================
/*!
@param images
*/
//============================================================
void CInputImageView::onImagesSelected(std::vector<uint32_t>& images)
{
  mDataID = images;

  updateView();
}

void CInputImageView::updateView()
{
  std::vector<std::tuple<uint32_t, QImage&>> updatedView;

  for(uint32_t i : mDataID)
  {
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> j : *appliedData->getInputImages())
    {
      if(std::get<0>(j) == i)
      updatedView.push_back(std::tuple<uint32_t, QImage&>(i, std::get<1>(j)));
    }
  }

  showImages(updatedView);
}
