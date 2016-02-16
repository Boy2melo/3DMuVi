#ifndef C3DVIEW_H
#define C3DVIEW_H

#include <QWidget>
#include <QComboBox>

#include <workflow/workflow/idataview.h>

#include <gui/IGuiDataView.h>

#include "CPclView.h"

class C3dView : public CPclView, public IGuiDataView, public IDataView
{
  Q_OBJECT

  enum E3dModelType
  {
    POINT_CLOUD,
    MESH,
    TEXTURED
  };

public:
  //TODO: add combo box to select model type
  C3dView(); //QComboBox& modelTypeComboBox);
  void activate();
  //TODO: Include  data packets
  //void applyData(CPclDataPacket* packet);
  //void applyData(CPoseDataPacket* packet);

public slots:
  void onCurrentIndexChangedModelType(int index);
};

#endif // C3DVIEW_H
