#ifndef CDATAVIEWTABCONTAINER_H
#define CDATAVIEWTABCONTAINER_H

#include "CImagePreviewWidget.h"

#include <QTabWidget>

class CDataViewTabContainer : public QTabWidget
{
public:
    CDataViewTabContainer(CImagePreviewWidget* imagePreview);

private slots:
    void onCurrentChanged(int index);
};

#endif // CDATAVIEWTABCONTAINER_H
