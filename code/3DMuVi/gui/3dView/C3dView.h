#ifndef C3DVIEW_H
#define C3DVIEW_H

#include <QWidget>
#include <QComboBox>

#include <workflow/workflow/idataview.h>
#include <workflow/workflow/datapackets/CDataFusion.h>
#include <workflow/workflow/datapackets/CDataPose.h>

#include <gui/IGuiDataView.h>

#include "CPclView.h"

/*!
\brief A widget presenting a point cloud and camera poses.
\author Stefan Wolf

This class is a widget which presents a point cloud and camera poses. It uses the pcl visualizer
for drawing.
*/
class C3dView : public CPclView, public IGuiDataView, public IDataView
{
  Q_OBJECT

  /*!
  \brief Defines possible types of model types to present
  */
  enum E3dModelType
  {
    POINT_CLOUD,
    MESH,
    TEXTURED
  };

public:
  /*!
  \brief Initializes the object.
  \param parent The parent of this widget.
  */
  C3dView(QWidget* parent = nullptr);

  /*!
  \brief Sets the combo box for the model type selection.
  \param modelTypeComboBox A combo box to select the model type to show.

  This method adds the possible model types to the combo box and connects it to this class so that
  the selected model will be shown when the user changes the selection.
  */
  void setModelTypeSelector(QComboBox* modelTypeComboBox);

  /*!
  \brief Activates the widget.

  This method sends out the signal relevantImagesChanged with the poses as image IDs.
  */
  void activate();

  /*!
  \brief Shows the given fusion data.
  \param packet The data to show.

  Displays the model which is in the given data packet.
  */
  void applyData(CDataFusion* packet);

  /*!
  \brief Shows the given pose data.
  \param packet The data to show.

  Displays the given poses as camera meshes.
  */
  void applyData(CDataPose* packet);

signals:
  /*!
  \brief Emitted when activate is called.
  \param images Relevant images IDs.

  This signal indicates that a new set of images is relevant for the user now.
  */
  void relevantImagesChanged(std::vector<uint32_t>& images);

private slots:
  void onCurrentIndexChangedModelType(int index);

private:
  bool mCloudAdded = false;
  std::vector<uint64_t> mPoseIds;
};

#endif // C3DVIEW_H
