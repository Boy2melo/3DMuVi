#include "CInputImageView.h"

#include "io/CInputDataSet.h"

//============================================================
/*!
@param packet
*/
//============================================================
void CInputImageView::applyData(const CInputDataSet* packet)
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
  if(appliedData)
  {
      std::vector<uint32_t> images;
      for(std::tuple<uint32_t, QImage, CImagePreviewItem> i : *appliedData->getInputImages())
      {
        images.push_back(std::get<0>(i));
      }
      emit relevantImagesChanged(images);
  }
}

void CInputImageView::clearData()
{

  std::vector<std::tuple<uint32_t, QImage&>> clearList;
  showImages(clearList);

  appliedData = nullptr;
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

  showImages(std::vector<std::tuple<uint32_t,QImage&>>());

  for(QImage* i : mImageList)
  {
    delete i;
  }
  mImageList.clear();

  for(uint32_t i : mDataID)
  {
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> j : *appliedData->getInputImages())
    {
      if(std::get<0>(j) == i)
      {
      QImage* newImage = new QImage(std::get<1>(j));
      mImageList.push_back(newImage);
      updatedView.push_back(std::tuple<uint32_t, QImage&>(i, *newImage));
      }
    }
  }

  showImages(updatedView);
}
