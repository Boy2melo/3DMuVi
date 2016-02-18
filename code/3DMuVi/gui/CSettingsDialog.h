#ifndef CSETTINGSDIALOG_H
#define CSETTINGSDIALOG_H

#include <QDialog>

namespace Ui {
  class CSettingsDialog;
}

/*!
\brief A dialog for managing global settings.
\author Stefan Wolf

This class provides a dialog in which the user can change global settings.
*/
class CSettingsDialog : public QDialog
{
    Q_OBJECT

public:
  //============================================================
  /*!
  \brief Initializes the dialog.

  This constructor initializes the dialog.
  @param parent The widget which should be the parent of this one.
  */
  //============================================================
  explicit CSettingsDialog(QWidget *parent = 0);

  //============================================================
  /*!
  \brief Cleans up the dialog.

  Destroys all created objects.
  */
  //============================================================
  ~CSettingsDialog();

public slots:
  //============================================================
  /*!
  \brief Applies the settings.

  This method closes the dialog and applies the settings.
  */
  //============================================================
  void accept();

  //============================================================
  /*!
  \brief Shows a directory selection dialog for the result directory.
  Shows a directory selection dialog for the result directory.

  @param checked Parameter will be ignored.
  */
  //============================================================
  void onResultDirectoryButtonClicked(bool checked);

private:
  Ui::CSettingsDialog *ui;
};

#endif // CSETTINGSDIALOG_H
