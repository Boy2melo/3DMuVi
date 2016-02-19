#ifndef CQJSONMODEL_H
#define CQJSONMODEL_H

#include <QAbstractItemModel>
#include "CQJsonItem.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QIcon>

#include <QUrl>
#include <QVector>
#include <QPair>
#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingController.h>
class CQJsonModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    /*!
      * \brief CQJsonModel constructs a new qjsonmodel
      * \param parent parent of the model
      * \param list vector with all the QJsonObject for the model
      */
     explicit CQJsonModel(QObject *parent = 0, QVector<QJsonObject> list = QVector<QJsonObject>());
     /*!
      * \brief saveSettings saves an Algorithmsetting as Jsonfile
      * \param row which setting will be saved
      * \param filename url for the file
      */
     void saveSettings(int row, QUrl filename);
     /*!
      * \brief loadSettings loads an algorithmsettingform a Jsonfile
      * \param row where the new setting should go, delets the old ons
      * \param filename url for the file
      */
     void loadSettings(int row, QUrl filename);

     Qt::ItemFlags flags(QModelIndex& index);
     bool setData(QModelIndex& index, QVariant& value, int role);

    bool load(const QString& fileName);
    bool load(QIODevice * device);
    bool loadJson(const QByteArray& json);
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    QModelIndex index(int row, int column,const QModelIndex &parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex &index) const;
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    void setIcon(const QJsonValue::Type& type, const QIcon& icon);

public slots:
    /*!
     * \brief loadQJson loads a qjsonobject into the model
     * \param data the QJsonObject
     */
    void loadQJson(QJsonObject data);
signals:
    void requestQJson(QUrl directory);
    void saveQJson(QJsonObject data, QUrl directory);

protected:
    CQJsonTreeItem backtrack(QModelIndex &index);

    CQJsonTreeItem * mRootItem;
    QJsonDocument mDocument;
    QStringList mHeaders;
    QHash<QJsonValue::Type, QIcon> mTypeIcons;


};

#endif // CQJSONMODEL_H
