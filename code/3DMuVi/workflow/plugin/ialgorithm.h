#ifndef IALGORITHM_H
#define IALGORITHM_H

#include <QStringList>
#include <QObject>
#include <QJsonObject>
#include <QPluginLoader>
#include "workflow/workflow/ccontextdatastore.h"
#include "logger/controll/CLogController.h"
#include "settings/CAlgorithmSettingController.h"
#include <functional>

/*!
 * \class IAlgorithm
 * \brief The IAlgorithm class
 * \author Nathanael Schneider, Stefan Wolf
 *
 * This interface declares methods which are not specific for a certain algorithm type. A
 * developer who wants to create a new plugin should not implement this interface directly.
 * Instead the interface for the certain algorithm type needs to be implemented. To make the
 * implementation easier, the use of AAlgorithmConfig is recommended which already has some
 * methods impelemented.
 *
 * Additionally, each plugin needs to export a symbol, which depends on the certain algorithm type,
 * to create a new object of the algorithm class. The name of the symbol is given by the
 * <i>symbol</i> variable of the algorithm type's interface.
 */
class IAlgorithm
{
public:
  /*!
   * \brief Empty destructor which should be overriden by the implementing class.
   */
  virtual ~IAlgorithm() { }

  /*!
   * \brief Return the plugin's author.
   * \return The plugin's author.
   */
  virtual QString getAuthor() const = 0;
  /*!
   * \brief Return the plugin's name.
   * \return The plugin's name.
   */
  virtual QString getName() const = 0;
  /*!
   * \brief Returns the date of the last change of the plugin.
   * \return The date of the last change of the plugin.
   */
  virtual QDate getDate() const = 0;
  /*!
   * \brief Returns the plugin's version number.
   * \return The plugin's verison number.
   */
  virtual int32_t getVersion() const = 0;

  /*!
   * \brief Returns the plugin's parameters with their default settings.
   * \return The plugin's parameters with their default settings. The keys should be the name of the
   * parameters and the values should be the default values.
   */
  virtual QJsonObject getParameterJson() const = 0;

  //TODO: define the format of the parameter's description and rework this function's documentation
  /*!
   * \brief Returns the description of the plugin's parameters.
   * \return The description of the plugin's parameters. The format of this json object is not
   * defined at the moment.
   */
  virtual QJsonObject getParameterDescriptionJson() const = 0;

  /*!
  \brief Checks whether the given settings are valid.
  \param settings The parameters which should be checked.
  \return True if all values are appropriate for the plugin. False otherwise.
  */
  virtual bool validateParameters(QJsonObject settings) const = 0;

  /*!
   * \brief Sets the logger which should be used by the plugin.
   */
  virtual void setLogger(CLogController* controller) = 0;
  /*!
   * \brief Sets the parameters which should be used by the plugin.
   */
  virtual void setParameters(QJsonObject settings) = 0;
  /*!
   * \brief Executes the plugin's algorithm on the previously set data. Look at the interface of the
   * certain algorithm type to see which data the algorithm receives as input and which data he needs to
   * provide as output.
   */
  virtual bool run() = 0;

  /*!
   * \brief Checks whether the algorithm is currently busy.
   * \return True if the algorithm is busy. False otherwise.
   */
  virtual bool IsBusy() const = 0;
};

#endif // IALGORITHM_H
