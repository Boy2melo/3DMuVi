#ifndef CIMAGEPREVIEWWIDGET_H
#define CIMAGEPREVIEWWIDGET_H

#include <QListWidget>

#include "CImagePreviewItem.h"
/*!
 * \brief The widget for the image preiview.
 * \author Grigori Schapoval
 *
 */
class CImagePreviewWidget : public QListWidget
{
  Q_OBJECT

public: 
  explicit CImagePreviewWidget(QWidget* parent = nullptr);
  /*!
   * \brief sets Images
   * \param images
   */
  void setImages(std::vector<CImagePreviewItem*> images);

public slots:

  /*!
   * \brief changes the Images.
   * \param images
   */
  void onRelevantImagesChanged(std::vector<uint32_t>& images);
    
signals:
  /*!
   * \brief This signal is emitted when the images change.
   * \param images
   */
  void imagesSelected(std::vector<uint32_t>& images);

private slots:
  void onItemSelectionChanged();

private:
  std::vector<CImagePreviewItem*> mImages;
};

#endif // CIMAGEPREVIEWWIDGET_H
