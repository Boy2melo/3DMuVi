#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "workflow/plugin/ifeaturematcher.h"
#include "workflow/plugin/aalgorithmconfig.h"
#include <QJsonObject>
#include <QObject>

class PluginSurf : public AAlgorithmConfig<PluginSurf, IFeatureMatcher>
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject mSettings;

    std::shared_ptr<CInputDataSet> mImages;
    std::shared_ptr<CDataFeature> mFeatures;

public:
    static constexpr auto author = "Bastian Erdnuess";
    static constexpr auto name = "OpenSurf";
    static constexpr auto date = "2016-02-18";
    static constexpr int32_t version = 1;
    static constexpr auto jsonPath = "PluginSurf.json";

    PluginSurf();
    ~PluginSurf();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    virtual void setLogger(CLogController *controller) override;

    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    virtual void setParameters(QJsonObject settings) override;
    /*!
     * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    virtual bool run() override;

    /*!
    \brief Gibt zurück, ob der Algorithmus zur Zeit mit einer Ausführung beschäftigt ist
    */
    virtual bool IsBusy() const override;

    virtual void setImages(std::shared_ptr<CInputDataSet> images) override;

    virtual std::shared_ptr<CDataFeature> getFeatureMatches() override;

    static std::shared_ptr<IFeatureMatcher> newFeatureMatcher();

protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm();
};

#endif // ALGORITHM_H
