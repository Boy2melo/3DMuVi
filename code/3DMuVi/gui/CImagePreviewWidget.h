#ifndef CIMAGEPREVIEWWIDGET_H
#define CIMAGEPREVIEWWIDGET_H

#include <QListWidget>

#include "CImagePreviewItem.h"

class CImagePreviewWidget : public QListWidget
{
  Q_OBJECT

public: 
  explicit CImagePreviewWidget(QWidget* parent = nullptr);
  void setImages(std::vector<CImagePreviewItem*> images);

public slots:
  void onRelevantImagesChanged(std::vector<uint32_t>& images);
    
signals:
  void imagesSelected(std::vector<uint32_t>& images);

private slots:
  void onItemSelectionChanged();

private:
  std::vector<CImagePreviewItem*> mImages;
};

#endif // CIMAGEPREVIEWWIDGET_H
