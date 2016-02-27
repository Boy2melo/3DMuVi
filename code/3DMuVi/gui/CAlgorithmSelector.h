#ifndef CALGORITHMSELECTOR_H
#define CALGORITHMSELECTOR_H

#include <QWidget>
#include <QObject>
#include <workflow/workflow/aworkflow.h>

/*!
 *
 * \brief A dialog for managing global settings.
 * \author Grigori Schapoval
 *
 * This class provides a dialog in which the user can change global settings.
 */

class CAlgorithmSelector : public QWidget
{
  Q_OBJECT

public:
  explicit CAlgorithmSelector(QWidget *parent);
  ~CAlgorithmSelector();
  /*!
   * \brief sets one of the possible workflow.
   * \param workflow
   */
  void setWorkflow(AWorkflow& workflow);
  /*!
   * \brief setDataStore
   * \param storeId
   *
   * Sets the Datastore for the workflow.
   */
  void setDataStore(const QString& storeId);

signals:
  /*!
   * \brief This signal is emitted when the workflows execution state changes.
   * \param step
   */
  void algorithmChanged(int step);
  /*!
   * \brief This signal is emitted when the workflow is running.
   * \param isRunning
   */
  void workflowRunning(bool isRunning);

private:
  AWorkflow* mpWorkflow = nullptr;
  QString mDataStoreId;
private slots:
  void onCurrentIndexChanged(int index);
  void startButtonPushed(bool isPushed);
  void onDataStoreFinished(CContextDataStore* dataStore);
};

#endif // CALGORITHMSELECTOR_H
