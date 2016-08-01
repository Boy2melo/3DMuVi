#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "workflow/plugin/ifusor.h"
#include "workflow/plugin/aalgorithmconfig.h"
#include <QJsonObject>
#include <QObject>
#include <fstream>

#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

class CPluginDepthFusor : public AAlgorithmConfig<CPluginDepthFusor, IFusor>
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject mSettings;

    std::shared_ptr<CInputDataSet> mImages;
    std::shared_ptr<CDataDepth> mDepthMaps;
    std::shared_ptr<CDataFusion> mFusion;

    /*!
     * \brief File stream of groundtruth poses.
     */
    std::fstream  mPoseFileStream;

public:
    static constexpr auto author = "Boitumelo Ruf";
    static constexpr auto name = "DepthFusor";
    static constexpr auto date = "2016-02-18";
    static constexpr int32_t version = 1;
    static constexpr auto jsonPath = "PluginDepthFusor.json";

    CPluginDepthFusor();
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
    void setDepthMaps(std::shared_ptr<CDataDepth> depthMaps) override;

    std::shared_ptr<CDataFusion> getFusion() override;

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    virtual bool validateParameters(QJsonObject settings) const override;

    static std::shared_ptr<IFusor> newFusor();

protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm();

private:
    /*!
     * \brief Method to load pcl polygon mesh from ply.
     * \param iFilePath Path to file.
     * \return Polygon Mesh pointer.
     */
    pcl::PolygonMesh::Ptr loadMeshFromPly(const std::string &iFilePath);

    /*!
     * \brief Method to load pcl point cloud from pcd file.
     * \param iFilePath Path to source file.
     * \return Point Cloud Pointer
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCoudFromPcd(const std::string &iFilePath);
};

#endif // ALGORITHM_H
