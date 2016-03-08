#ifndef CTESTALGORITHMSETTINGS_H
#define CTESTALGORITHMSETTINGS_H

#include <QTest>

/*!
\brief Testing algorithm settings in connection with workflow.
\author Stefan Wolf

This test is specified as user interface test T3.5 in the specification sheet. It requires the
algorithm settings test plugin.
*/
class CTestAlgorithmSettings : public QObject
{
  Q_OBJECT

public:
  /*!
  \brief Should be called by the algorithm settings test plugin.
  \param settings The settings passed to the plugin.

  This method should be called by the algorithm settings test plugin when
  IAlgorithm::setParameters is called. It copys the settings pointer to a private static variable
  in this class.
  */
  static void setParameters(QJsonObject* settings);

private slots:
  /*!
  \brief Runs the test.

  This test creates a CAlgorithmSettingsView and a four-phase workflow. After the creation it sets
  the algorithm settings test plugin as first plugin and changes the plugins parameter to 33. Then
  it checks if the change has been applied to the plugin. Afterwards it sets the parameter to 66
  and checks it again. This time the change should be denied since only values below 50 are
  allowed.
  */
  void test();

private:
  static QJsonObject* mpSettings;
  static const int TEST_VALUE_VALID;
  static const int TEST_VALUE_INVALID;
};

#endif // CTESTALGORITHMSETTINGS_H
