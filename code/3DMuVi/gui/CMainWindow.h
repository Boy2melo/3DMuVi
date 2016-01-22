#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>

class CMainWindow : public QMainWindow
{
private slots:
    void onSaveWorkflow(bool checked);
    void onSaveWorkflowAs(bool checked);
    void onLoadImages(bool checked);
    void onAdvancedLoadFiles(bool checked);
    void onWorkflowSelected(bool checked);
    void onSettings(bool checked);
    void onAbout(bool checked);
};

#endif // CMAINWINDOW_H
