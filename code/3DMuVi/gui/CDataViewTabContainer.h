#ifndef CDATAVIEWTABCONTAINER_H
#define CDATAVIEWTABCONTAINER_H

#include <QTabWidget>

#include <workflow/workflow/ccontextdatastore.h>

#include "3dView/C3dView.h"
#include "ImageViews/CDepthMapView.h"
#include "ImageViews/CFeatureView.h"
#include "ImageViews/CInputImageView.h"

#include "CImagePreviewWidget.h"

/*!
\brief This class contains all views
\author Stefan Wolf

This class contains all views.
*/
class CDataViewTabContainer : public QTabWidget
{
  Q_OBJECT

public:
  /*!
  \brief This class' constructor.
  \param parent This widgets parent.

  Creates all views.
  */
  explicit CDataViewTabContainer(QWidget* parent = nullptr);

  /*!
  \brief Sets the image preview widget.
  \param imagePreview The image preview widget to use.

  The given image preview widget will be connected to the contained views.
   */
  void setImagePreviewWidget(CImagePreviewWidget* imagePreview);

  /*!
  \brief Updates the shown data.
  \param dataStorage The data storage which should be displayed.

  Updates all contained views to display data storages data.
   */
  void applyDataStorage(CContextDataStore* dataStorage);

private slots:
  void onCurrentChanged(int index);

private:
  CInputImageView* mpInputImageView = nullptr;
  CFeatureView* mpFeatureView = nullptr;
  CDepthMapView* mpDepthMapView = nullptr;
  C3dView* mp3dView = nullptr;
};

#endif // CDATAVIEWTABCONTAINER_H
