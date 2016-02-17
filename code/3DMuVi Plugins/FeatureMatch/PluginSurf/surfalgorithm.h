#ifndef EXAMPLEALGORITHM_H
#define EXAMPLEALGORITHM_H

#include <QImage>
#include <QJsonObject>
#include <QObject>

#include "workflow/plugin/ialgorithm.h"
//#include "workflow/workflow/datapackets/CDataInputImages.h"
#include "workflow/workflow/datapackets/CDataFeature.h"

class SurfAlgorithm : public IAlgorithm
{
    Q_OBJECT
private:
    bool mIsBusy = false;
    CLogController *mLogger = nullptr;
    QJsonObject *mSettings = nullptr;

public:
    SurfAlgorithm();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(CLogController *controller) override;
    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    void setParameters(QJsonObject *settings) override;
    /*!
     * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    virtual void run(CContextDataStore* dataStore, std::function<void (CContextDataStore*)> callback) override;

    /*!
    \brief Gibt zurück, ob der Algorithmus zur Zeit mit einer Ausführung beschäftigt ist
    */
    virtual bool IsBusy() const override;

    /*!
    * \brief Eine Liste aller Daten, die als Eingabe benötigt werden.
    * \return Eine Liste aller Daten, die als Eingabe benötigt werden.
    */
    virtual QStringList getInputDataTypes() const override;
    /*!
    * \brief Eine Liste aller Daten, die als Ausgabe erzeugt werden.
    * \return Eine Liste aller Daten, die als Ausgabe erzeugt werden.
    */
    virtual QStringList getOutputDataTypes() const override;

  private:

    void run(CContextDataStore* dataStore);
};

// mock up:

#define DT_INPUT_IMAGES "Input Images"

using InputImages = std::vector<QImage>;
struct CDataInputImages
{
    InputImages getImages() { return std::vector<QImage>(); }
};

struct CDataFeatureMatch
{
    void setFeatureMatch(FeatureMatch &&) { }
};

template <typename T>
T * CContextDataStore::getData()
{
  return nullptr;
}


#endif // EXAMPLEALGORITHM_H
