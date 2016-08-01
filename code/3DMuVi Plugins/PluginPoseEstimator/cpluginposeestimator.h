#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <fstream>
#include "workflow/plugin/iposeestimator.h"
#include "workflow/plugin/aalgorithmconfig.h"
#include <QJsonObject>
#include <QObject>


class CPluginPoseEstimator : public AAlgorithmConfig<CPluginPoseEstimator, IPoseEstimator>
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject mSettings;
    std::fstream mPoseFileStream;

    std::shared_ptr<CInputDataSet> mImages;
    std::shared_ptr<CDataFeature> mFeatures;
    std::shared_ptr<CDataPose> mPoses;

public:
    static constexpr auto name = "PoseEstimator";
    static constexpr auto author = "Boitumelo Ruf";
    static constexpr auto date = "2016-02-18";
    static constexpr int32_t version = 1;
    static constexpr auto jsonPath = "PluginPoseEstimator.json";

    CPluginPoseEstimator();
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

    void setImages(std::shared_ptr<CInputDataSet> images) override;
    void setFeatureMatches(std::shared_ptr<CDataFeature> featureMatches) override;

    std::shared_ptr<CDataPose> getPoses() override;

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    virtual bool validateParameters(QJsonObject settings) const override;

    static std::shared_ptr<IPoseEstimator> newPoseEstimator();

protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm();

private:
    void openFileStream();

    void cvtEulerToQuaternion(double const iAngleX, double const iAngleY, double const iAngleZ,
                              double & oQW, double& oQX, double& oQY, double& oQZ);

};
#endif // ALGORITHM_H
