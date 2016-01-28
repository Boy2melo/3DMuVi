#ifndef CIMAGEPREVIEWITEM_H
#define CIMAGEPREVIEWITEM_H

#include <QListWidgetItem>

class CImagePreviewItem: public QListWidgetItem
{
public:
  CImagePreviewItem(const QIcon& icon, const QString& text, uint32_t imageId);
  uint32_t getImageId();
};

#endif // CIMAGEPREVIEWITEM_H
