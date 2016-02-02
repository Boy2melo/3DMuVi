#include "CImagePreviewItem.h"

//============================================================
/*!
@param icon
@param text
@param imageId
*/
//============================================================
CImagePreviewItem::CImagePreviewItem(const QIcon& icon, const QString& text, uint32_t imageId) :
  QListWidgetItem(icon, text), mImageId(imageId)
{

}

//============================================================
/*!
@return uint32_t
*/
//============================================================
uint32_t CImagePreviewItem::getImageId()
{
    return mImageId;
}
