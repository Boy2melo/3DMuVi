#ifndef CIMAGEPREVIEWITEM_H
#define CIMAGEPREVIEWITEM_H

#include <QListWidgetItem>
/*!
 * \brief The the class for pictures in picture preview.
 */
class CImagePreviewItem: public QListWidgetItem
{
public:
  CImagePreviewItem(const QIcon& icon, const QString& text, uint32_t imageId);
  uint32_t getImageId();

private:
  uint32_t mImageId;
};

#endif // CIMAGEPREVIEWITEM_H
