#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "workflow/plugin/ialgorithm.h"
#include "plugin_config.h"
#include <QJsonObject>
#include <QObject>
#include <fstream>

#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

class _CLASS_GEN(Algorithm) : public IAlgorithm
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject *mSettings;
    QStringList mInputTypes;
    QStringList mOutputTypes;

    /*!
     * \brief File stream of groundtruth poses.
     */
    std::fstream  mPoseFileStream;

public:
    _CLASS_GEN(Algorithm)();
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

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    bool ValidateParameters(const QJsonObject*) const;
protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm(CContextDataStore* store);

    /*!
     * \brief Wird im Konstruktor aufgerufen. Fülle mInputTypes und mOutputTypes
     */
    virtual void OnInitialize();

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
