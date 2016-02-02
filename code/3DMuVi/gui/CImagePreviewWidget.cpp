#include <QMessageBox>

#include "CImagePreviewWidget.h"

//============================================================
/*!
@param images
*/
//============================================================
CImagePreviewWidget::CImagePreviewWidget(QWidget* parent) : QListWidget(parent)
{
  setViewMode(QListWidget::IconMode);
  //setIconSize(QSize(200,200));
  setResizeMode(QListWidget::Adjust);
  connect(this, &CImagePreviewWidget::itemSelectionChanged, this, &CImagePreviewWidget::onItemSelectionChanged);
}

//============================================================
/*!
@param images
*/
//============================================================
void CImagePreviewWidget::setImages(std::vector<CImagePreviewItem*> images)
{
  mImages = images;
  clear();
  for(CImagePreviewItem* i : images)
  {
    addItem(i);
  }
}

//============================================================
/*!
@param images
*/
//============================================================
void CImagePreviewWidget::onRelevantImagesChanged(std::vector<uint32_t>& images)
{
  QMessageBox::warning(nullptr, "Atemlos durch die Nacht", "Atemlos durch die Nacht");

  clear();
  for(uint32_t i : images)
  {
    for(CImagePreviewItem* j : mImages)
    {
      if(i == j->getImageId())
      {
        addItem(j);
      }
    }
  }
}

//============================================================
/*!
@param images
*/
//============================================================

void CImagePreviewWidget::onItemSelectionChanged()
{
  QList<QListWidgetItem*> sellection = selectedItems();
  std::vector<uint32_t> images;

  for(QListWidgetItem* i : sellection)
  {
    CImagePreviewItem* item = dynamic_cast<CImagePreviewItem*>(i);
    if(item)
    {
      images.push_back(item->getImageId());
    }
  }

  emit imagesSelected(images);
}
