#ifndef AALGORITHMCONFIG_H
#define AALGORITHMCONFIG_H

#include <QString>
#include <QDate>
#include <QJsonDocument>

/*!
 * \brief This class offers an easier way to implement plugins than by implementig IAlgorithm
 * directly.
 * \author Stefan Wolf
 *
 * This class implements methods required for implementing IAlgorithm. To make use of this class,
 * the developer should pass the plugin's class as first template parameter and the algorithm type's
 * interface as second template parameter. Moreover, the plugin's class should then derive from this
 * class. Additionally, the plugin's class need to offer the following variables: author, name,
 * date, version, jsonPath. Author, name, date and jsonPath need to be of a type which can be
 * implicitly converted to QString. Date should be the date of the last change made to the plugin
 * and needs to be in the format yyyy-MM-dd. Version should be an integer. jsonPath should be a
 * path to the plugin's config file which has to be in json format and should contain the keys
 * <i>Default</i> and <i>Description</i> with json objects as values. The format of default is
 * defined in the documentation of IAlgorithm::getParameterJson and the description's format at
 * IAlgorithm::getParameterDescriptionJson.
 *
 * If used appropriately this class derives from the algorithm type's interface and implements
 * the methods IAlgorithm::getAuthor, IAlgorithm::getName, IAlgorithm::getDate,
 * IAlgorithm::getVersion, IAlgorithm::getParameterJson, IAlgorithm::getParameterDescriptionJson and
 * IAlgorithm::validateParameters.
 */
template <typename T, typename TAlgorithm>
class AAlgorithmConfig : public TAlgorithm
{
public:
  virtual ~AAlgorithmConfig() { }

  virtual QString getAuthor() const override
  {
    return T::author;
  }

  virtual QString getName() const override
  {
    return T::name;
  }

  virtual QDate getDate() const override
  {
    return QDate::fromString(T::date, "yyyy-MM-dd");
  }

  virtual int32_t getVersion() const override
  {
    return T::version;
  }

  /*!
   * \brief Reads the json file at T::jsonPath and returns the object with key <i>Default</i>.
   * \return The plugin's parameters with default values.
   */
  virtual QJsonObject getParameterJson() const override {
    QFile json(T::jsonPath);
    if(!json.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      return QJsonObject();
    }
    return QJsonDocument::fromJson(json.readAll()).object().value("Default").toObject();
  }

  /*!
   * \brief Reads the json file at T::jsonPath and returns the object with key <i>Description</i>.
   * \return The plugin parameter's description.
   */
  virtual QJsonObject getParameterDescriptionJson() const override {
    QFile json(T::jsonPath);
    if(!json.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      return QJsonObject();
    }
    return QJsonDocument::fromJson(json.readAll()).object().value("Description").toObject();
  }

  /*!
   * \brief Checks whether all given parameters match the type of the plugin's default values.
   * \return True if all parameters have the same type as the default values. False otherwise.
   */
  virtual bool validateParameters(QJsonObject params) const override {
    // First level type check over given parameters
    for(QJsonObject::iterator itr = params.begin(); itr != params.end(); itr++){
        // Get reference type
        auto ref = getParameterJson().value(itr.key());
        if(ref.type() != itr.value().type()) {
          return false;
        }
    }

    return true;
  }
};

#endif // AALGORITHMCONFIG_H
