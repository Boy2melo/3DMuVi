#include <QVBoxLayout>

#include <QScrollArea>

#include "CDataViewTabContainer.h"

CDataViewTabContainer::CDataViewTabContainer(QWidget* parent) : QTabWidget(parent)
{
#ifdef PCL
  QWidget* container3dView = new QWidget(this);
  QWidget* containerModelTypeSelector = new QWidget(container3dView);
  QBoxLayout* layoutModelTypeSelector = new QHBoxLayout;
  QComboBox* modelTypeSelector = new QComboBox(containerModelTypeSelector);
#endif
  QScrollArea* inputImageViewArea = new QScrollArea(this);
  QScrollArea* featureViewArea = new QScrollArea(this);
  QScrollArea* depthMapViewArea = new QScrollArea(this);

  mpInputImageView = new CInputImageView;
  inputImageViewArea->setWidget(mpInputImageView);
  mpInputImageView->setScrollBars(inputImageViewArea->horizontalScrollBar(),
                                  inputImageViewArea->verticalScrollBar());
  addTab(inputImageViewArea, "Input images");

  mpFeatureView = new CFeatureView;
  featureViewArea->setWidget(mpFeatureView);
  mpFeatureView->setScrollBars(featureViewArea->horizontalScrollBar(),
                               featureViewArea->verticalScrollBar());
  addTab(featureViewArea, "Features");

  mpDepthMapView = new CDepthMapView;
  depthMapViewArea->setWidget(mpDepthMapView);
  mpDepthMapView->setScrollBars(depthMapViewArea->horizontalScrollBar(),
                                depthMapViewArea->verticalScrollBar());
  addTab(depthMapViewArea, "Depth maps");

#ifdef PCL
  containerModelTypeSelector->setLayout(layoutModelTypeSelector);
  layoutModelTypeSelector->addWidget(modelTypeSelector);
  layoutModelTypeSelector->addStretch();
  container3dView->setLayout(new QVBoxLayout);
  container3dView->layout()->addWidget(containerModelTypeSelector);
  mp3dView = new C3dView;
  mp3dView->setModelTypeSelector(modelTypeSelector);
  container3dView->layout()->addWidget(mp3dView);
  addTab(container3dView, "3D view");
#endif

  connect(this, &QTabWidget::currentChanged, this, &CDataViewTabContainer::onCurrentChanged);
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

#ifdef PCL
  connect(mp3dView, &C3dView::relevantImagesChanged, imagePreview,
          &CImagePreviewWidget::onRelevantImagesChanged);
#endif
}

void CDataViewTabContainer::applyDataStorage(CContextDataStore* dataStorage)
{
  dataStorage->applyToDataView(mpInputImageView);
  dataStorage->applyToDataView(mpFeatureView);
  dataStorage->applyToDataView(mpDepthMapView);

#ifdef PCL
  dataStorage->applyToDataView(mp3dView);
#endif

  onCurrentChanged(currentIndex());
}

void CDataViewTabContainer::clearData()
{
  mpInputImageView->clearData();
  mpFeatureView->clearData();
  mpDepthMapView->clearData();

#ifdef PCL
  mp3dView->clearData();
#endif
}

void CDataViewTabContainer::onCurrentChanged(int index)
{
  if(tabText(index) == "Input images")
  {
    mpInputImageView->activate();
  }
  else if(tabText(index) == "Features")
  {
    mpFeatureView->activate();
  }
  else if(tabText(index) == "Depth maps")
  {
    mpDepthMapView->activate();
  }
#ifdef PCL
  else if(tabText(index) == "3D view")
  {
    mp3dView->activate();
  }
#endif
}
