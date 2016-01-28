#ifndef CQJSONMODEL_H
#define CQJSONMODEL_H

#include <QAbstractItemModel>
#include "CQJsonItem.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QIcon>

#include <QUrl>
#include <QList>
#include <QPair>
#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingController.h>
class QJsonModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    /*!
      * \brief CQJsonModel
      * \param workflow
      * \param controller
      * \param parent
      */
     explicit CQJsonModel(AWorkflow& workflow, CAlgorithmSettingController& controller, QObject *parent = 0);
     /*!
      * \brief saveSettings
      * \param row
      * \param filename
      */
     void saveSettings(int row, QUrl filename);
     /*!
      * \brief loadSettings
      * \param row
      * \param filename
      */
     void loadSettings(int row, QUrl filename);
     /*!
      * \brief flags
      * \param index
      * \return
      */
     Qt::ItemFlags flags(QModelIndex& index);
     /*!
      * \brief setData
      * \param index
      * \param value
      * \param role
      * \return
      */
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



private:
    CAlgorithmSettingController algocontoller;
    QJsonTreeItem backtrack(QModelIndex &index);
    QList<QPair<QString, int>> algolist;

    QJsonTreeItem * mRootItem;
    QJsonDocument mDocument;
    QStringList mHeaders;
    QHash<QJsonValue::Type, QIcon> mTypeIcons;


};

#endif // CQJSONMODEL_H
