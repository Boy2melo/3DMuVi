#include "CDepthMapView.h"


#include "workflow/workflow/datapackets/CDataDepth.h"

//============================================================
/*!
@param packet
*/
//============================================================
void CDepthMapView::applyData(const CDataDepth* packet)
{
  appliedData = packet;

  updateView();
}

//============================================================
/*!
*/
//============================================================
void CDepthMapView::activate()
{
  std::vector<uint32_t> images;

  if(appliedData)
  {
    for(std::tuple<uint32_t, QImage> i : *appliedData->getDepthMap())
    {
      images.push_back(std::get<0>(i));
    }
  }

  emit relevantImagesChanged(images);
}

//============================================================
/*!
 * \brief CDepthMapView::clearData
 */
//============================================================

void CDepthMapView::clearData()
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
void CDepthMapView::onImagesSelected(std::vector<uint32_t>& images)
{
  mDataID = images;
  updateView();
}

void CDepthMapView::updateView()
{
  std::vector<std::tuple<uint32_t, QImage&>> updatedView;

  showImages(std::vector<std::tuple<uint32_t,QImage&>>());

  for(QImage* i : mImageList)
  {
    delete i;
  }
  mImageList.clear();

  if(appliedData != nullptr)
  {
    for(uint32_t i : mDataID)
    {
      for(std::tuple<uint32_t, QImage> j : *appliedData->getDepthMap())
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
}
