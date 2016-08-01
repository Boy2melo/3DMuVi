#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "workflow/plugin/idepthestimator.h"
#include "workflow/plugin/aalgorithmconfig.h"
#include <QJsonObject>
#include <QObject>

class CPluginDepthEstimator : public AAlgorithmConfig<CPluginDepthEstimator, IDepthEstimator>
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject mQJson;
    QJsonObject mSettings;

    std::shared_ptr<CInputDataSet> mImages;
    std::shared_ptr<CDataPose> mPoses;
    std::shared_ptr<CDataDepth> mDepthMaps;

    /*!
     * \brief List containing paths to groundtruth files within source directory.
     */
    QStringList               mFileList;

    /*!
     * \brief Iterator on mFileList.
     */
    QStringList::iterator     mFileItr;

public:
    static constexpr auto author = "Boitumelo Ruf";
    static constexpr auto name = "DepthEstimator";
    static constexpr auto date = "2016-02-18";
    static constexpr int32_t version = 1;
    static constexpr auto jsonPath = "PluginDepthEstimator.json";

    CPluginDepthEstimator();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(CLogController *controller) override;
    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    void setParameters(QJsonObject settings) override;
    /*!
     * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    virtual bool run() override;

    /*!
    \brief Gibt zurück, ob der Algorithmus zur Zeit mit einer Ausführung beschäftigt ist
    */
    virtual bool IsBusy() const override;

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    virtual bool validateParameters(QJsonObject params) const override;

    void setImages(std::shared_ptr<CInputDataSet> images) override;
    void setPoses(std::shared_ptr<CDataPose> poses) override;

    std::shared_ptr<CDataDepth> getDepthMaps() override;

    static std::shared_ptr<IDepthEstimator> newDepthEstimator();

protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm();
};

#endif // ALGORITHM_H
