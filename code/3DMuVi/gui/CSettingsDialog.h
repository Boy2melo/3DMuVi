#ifndef CSETTINGSDIALOG_H
#define CSETTINGSDIALOG_H

#include <QDialog>

class CSettingsDialog : public QDialog
{
public slots: 
  void accept();
  void onResultDirectoryButtonClicked(bool checked);
};

#endif // CSETTINGSDIALOG_H
