#include "CFeatureView.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "io/CInputDataSet.h"


//============================================================
/*!
@param packet
*/
//============================================================
void CFeatureView::applyData(CDataFeature* packet)
{
  appliedFeatureData = packet;

  updateView();
}

//============================================================
/*!
@param packet
*/
//============================================================
void CFeatureView::applyData(CInputDataSet* packet)
{
  appliedInputData = packet;

  updateView();
}

//============================================================
/*!
*/
//============================================================
void CFeatureView::activate()
{

  std::vector<uint32_t> images;
  for(std::tuple<uint64_t, float, float, uint32_t> i : *appliedFeatureData->getFeatureMatch())
  {
    if(std::find(images.begin(), images.end(), std::get<0>(i)) == images.end()) {
      images.push_back(std::get<0>(i));
    }
  }

  emit relevantImagesChanged(images);
}

//============================================================
/*!
@param images
*/
//============================================================
void CFeatureView::onImagesSelected(std::vector<uint32_t>& images)
{
    //BUG: missing implementation
}

void CFeatureView::updateView()
{
  std::vector<std::tuple<uint32_t, QImage&>> updatedView;
  std::map<uint32_t, std::vector<ImageFeature>> features;
  if(appliedInputData != nullptr && appliedFeatureData != nullptr)
  {
    for(uint32_t i : mDataID)
    {
      for(std::tuple<uint32_t, QImage, CImagePreviewItem> j : *appliedInputData->getInputImages())
      {
        if(std::get<0>(j) == i)
        updatedView.push_back(std::tuple<uint32_t, QImage&>(i, std::get<1>(j)));
      }
    }


    for(std::tuple<uint64_t, float, float, uint32_t> f : *appliedFeatureData->getFeatureMatch())
    {
      auto featureIterator = features.find(std::get<3>(f));
      ImageFeature newFeature(std::get<0>(f), QVector2D(std::get<1>(f), std::get<2>(f)));

      if(featureIterator == features.end())
      {
        std::vector<ImageFeature> newFeatureList;

        newFeatureList.push_back(newFeature);
        features.insert(std::pair<uint32_t, std::vector<ImageFeature>>
                        (std::get<3>(f), newFeatureList));

      }
      else
      {
        featureIterator->second.push_back(newFeature);
      }
    }

    for(auto f : features)
    {
      addConnectedMarkers(f.second);
    }
  }
}
