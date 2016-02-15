#include "CDataViewTabContainer.h"

CDataViewTabContainer::CDataViewTabContainer(QWidget* parent) : QTabWidget(parent)
{
  mpInputImageView = new CInputImageView;
  addTab(mpInputImageView, "Input images");

  mpFeatureView = new CFeatureView;
  addTab(mpFeatureView, "Features");

  mpDepthMapView = new CDepthMapView;
  addTab(mpDepthMapView, "Depth maps");

  mp3dView = new C3dView;
  addTab(mp3dView, "3D view");
}

void CDataViewTabContainer::setImagePreviewWidget(CImagePreviewWidget* imagePreview)
{
  connect(mpInputImageView, &CInputImageView::relevantImagesChanged, imagePreview,
          &CImagePreviewWidget::onRelevantImagesChanged);
  connect(imagePreview, &CImagePreviewWidget::imagesSelected, mpInputImageView,
          &CInputImageView::onImagesSelected);

  connect(mpFeatureView, &CFeatureView::relevantImagesChanged, imagePreview,
          &CImagePreviewWidget::onRelevantImagesChanged);
  connect(imagePreview, &CImagePreviewWidget::imagesSelected, mpFeatureView,
          &CFeatureView::onImagesSelected);

  connect(mpDepthMapView, &CDepthMapView::relevantImagesChanged, imagePreview,
          &CImagePreviewWidget::onRelevantImagesChanged);
  connect(imagePreview, &CImagePreviewWidget::imagesSelected, mpDepthMapView,
          &CDepthMapView::onImagesSelected);
}

void CDataViewTabContainer::applyDataStorage(CContextDataStore* dataStorage)
{
  dataStorage->ApplyToDataView(mpInputImageView);
  dataStorage->ApplyToDataView(mpFeatureView);
  dataStorage->ApplyToDataView(mpDepthMapView);
  dataStorage->ApplyToDataView(mp3dView);
}

void CDataViewTabContainer::onCurrentChanged(int index)
{
  IGuiDataView* dataView = dynamic_cast<IGuiDataView*>(widget(index));

  if(dataView)
  {
    dataView->activate();
  }
}
