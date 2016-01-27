#include "CImagePreviewWidget.h"

//============================================================
/*!
@param images
*/
//============================================================
CImagePreviewWidget::CImagePreviewWidget(QWidget* parent) : QListWidget(parent)
{
}

//============================================================
/*!
@param images
*/
//============================================================
void CImagePreviewWidget::setImages(std::vector<CImagePreviewItem*> images)
{

}

//============================================================
/*!
@param images
*/
//============================================================
void CImagePreviewWidget::onRelevantImagesChanged(std::vector<uint32_t>& images)
{
}

//============================================================
/*!
@param images
*/
//============================================================
void CImagePreviewWidget::imagesSelected(std::vector<uint32_t>& images)
{
}
