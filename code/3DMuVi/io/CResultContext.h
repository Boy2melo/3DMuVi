#ifndef CRESULTCONTEXT_H
#define CRESULTCONTEXT_H
#include <QUrl>
#include <QDir>
#include <QDateTime>
#include <QString>
#include <vector>

#include <workflow/workflow/idatapacket.h>
#include <logger/controll/CLogController.h>
#include <settings/CAlgorithmSettingController.h>
#include <settings/CGlobalSettingController.h>
#include <io/AStreamProvider.h>

/*!
 * \class CResultContext
 * \brief The CResultContext class
 * \author Laurenz Thiel
 *
 * Generates and represent the ResultContext on the drive.
 * The Directory consist of a log from the logController,
 * the algorithm settings and the global settings.
 * Furthermore, there will be one directory for each TDataPacket
 * type, which contains all the data from this type.
 */
class CResultContext {
public:
    /*!
     * \brief Constructor of the Class
     * \param path Path to the location wher the results should be saved.
     * \param algoSettings Provides the settings from the algorithms.
     * \param globalSettings Provides the global settings.
     * \return void
     */
    CResultContext(QUrl path,
        CAlgorithmSettingController* algoSettings,
        CGlobalSettingController* globalSettings);

    /*!
     * \brief The data will get serialized and accordingly to their TDataPacket
     * stored. The file is named after the id of the TDataPacket .
     * \param data TDataPacket which get stored.
     * \return void
     */
    void addDataPacket(std::shared_ptr<IDataPacket> data);

    /*!
     * \brief Returns a vector with all ids of TDataPackets which was found
     * in the result directory.
     * \return vector<QString> The vector which contains the ids.
     */
    std::vector<QString> getDataPacketIds();

    /*!
     * \brief Deserialize a file of the ResultContext.
     * \param id Select the file with this id.
     * \return TDataPacket of the deserialized file.
     */
    // NYI
    IDataPacket* getDataPacket(QString id);

private:
    QDir folder;
};

#endif // CRESULTCONTEXT_H
