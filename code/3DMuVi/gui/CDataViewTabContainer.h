#ifndef CDATAVIEWTABCONTAINER_H
#define CDATAVIEWTABCONTAINER_H

#include <QTabWidget>

#include <workflow/workflow/ccontextdatastore.h>

#include "CImagePreviewWidget.h"

class CDataViewTabContainer : public QTabWidget
{
public:
  explicit CDataViewTabContainer(QWidget* parent = nullptr);
  void setImagePreviewWidget(CImagePreviewWidget* imagePreview);
  void applyDataStorage(CContextDataStore* dataStorage);

private slots:
  void onCurrentChanged(int index);
};

#endif // CDATAVIEWTABCONTAINER_H
