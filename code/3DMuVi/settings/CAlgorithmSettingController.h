#ifndef CALGORITHMSETTINGCONTROLLER_H
#define CALGORITHMSETTINGCONTROLLER_H

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QUrl>
#include <QStringList>
#include "io/CTextIo.h"
/*!
* Controller for the parameters of the algortihms
* Author Jens Manig
*/
class CAlgorithmSettingController : public QObject
{
Q_OBJECT

private:
    QUrl tempdirectory;
    QStringList algorithms;
    CTextIo io;
protected:
    /*!
     * \gets a specific parameter of an algorithmn
     * \param name name of the algorithmn
     * \param key key for the parameter
     * \return the parameter
     */
    QJsonValue getSetting(QString name, QString key) const;
    /*!
     * \sets a parameter in an algorithmn to the specific value
     * \param name name of the algorithn
     * \param key key of the value
     * \param value new value
     * \return ture is legal value, else false
     */
    bool setSetting(QString name,QString key, QJsonValue value) const;
public:
    /*!
    * \Constructor for the Controller
    * \param directory temporal workdirectory
    */
    explicit CAlgorithmSettingController(QUrl directory);
    /*!
    * \Returns the setting for a certain algorithm
    * \param name name of the algorithm
    * \return the setting of the algorithm
    */
    QJsonObject* getSetting(QString name) const;
    /*!
    * \sets a new setting for an algorithm
    * \param name name of the algorithm
    * \param data the parameters of the algorithm as QJsonObject
    * \return true if data was a legal setting, else false
    */
    bool setSetting(QString name, QJsonObject data);
    /*!
    * \impoert a setting for an algorithm as QJsonObject
    *\param directory of the setting
    *\param name name of the setting
    */
    void import(QUrl directory, QString name);
    /*!
    * \exports all algorithmsettings to a directory
    *\param directory where all seeting should go
    */
    void exportTo(QUrl directory) const;
signals:
    void loadQJson(QJsonObject data);
public slots:
    /*!
     * \brief requestQJson loads a jsonfile from a directory, emits signal loadQJson with the load jsonobject
     * \param directory directory from where to load
     */
    void requestQJson(QUrl directory);
    /*!
     * \brief saveQJson saves a QJsonObject in a directory
     * \param data the QJsonObject
     * \param directory where the file should go
     */
    void saveQJson(QJsonObject data, QUrl directory);
};

#endif //CALGORITHMSETTINGCONTROLLER_H
