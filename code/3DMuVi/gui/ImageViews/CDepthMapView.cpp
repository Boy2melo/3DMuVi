#include "CDepthMapView.h"


#include "workflow/workflow/datapackets/CDataDepth.h"

//============================================================
/*!
@param packet
*/
//============================================================
void CDepthMapView::applyData(CDataDepth* packet)
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
  for(std::tuple<uint32_t, QImage> i : *appliedData->getDepthMap())
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
void CDepthMapView::onImagesSelected(std::vector<uint32_t>& images)
{
}

void CDepthMapView::updateView()
{
  std::vector<std::tuple<uint32_t, QImage&>> updatedView;

  for(uint32_t i : mDataID)
  {
    for(std::tuple<uint32_t, QImage> j : *appliedData->getDepthMap())
    {
      if(std::get<0>(j) == i)
      updatedView.push_back(std::tuple<uint32_t, QImage&>(i, std::get<1>(j)));
    }
  }
  showImages(updatedView);
}

