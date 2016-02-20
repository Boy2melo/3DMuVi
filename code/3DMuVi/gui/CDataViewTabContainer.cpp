#include <QVBoxLayout>

#include "CDataViewTabContainer.h"

CDataViewTabContainer::CDataViewTabContainer(QWidget* parent) : QTabWidget(parent)
{
  QWidget* container3dView = new QWidget(this);
  QWidget* containerModelTypeSelector = new QWidget(container3dView);
  QBoxLayout* layoutModelTypeSelector = new QHBoxLayout;
  QComboBox* modelTypeSelector = new QComboBox(containerModelTypeSelector);

  mpInputImageView = new CInputImageView;
  addTab(mpInputImageView, "Input images");

  mpFeatureView = new CFeatureView;
  addTab(mpFeatureView, "Features");

  mpDepthMapView = new CDepthMapView;
  addTab(mpDepthMapView, "Depth maps");

  containerModelTypeSelector->setLayout(layoutModelTypeSelector);
  layoutModelTypeSelector->addWidget(modelTypeSelector);
  layoutModelTypeSelector->addStretch();
  container3dView->setLayout(new QVBoxLayout);
  container3dView->layout()->addWidget(containerModelTypeSelector);
  mp3dView = new C3dView;
  mp3dView->setModelTypeSelector(modelTypeSelector);
  container3dView->layout()->addWidget(mp3dView);
  addTab(container3dView, "3D view");
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
