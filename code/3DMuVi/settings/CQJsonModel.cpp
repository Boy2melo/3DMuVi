/***********************************************
    Copyright (C) 2014  Schutz Sacha
    This file is part of QJsonModel (https://github.com/dridk/QJsonmodel).

    QJsonModel is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QJsonModel is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QJsonModel.  If not, see <http://www.gnu.org/licenses/>.

**********************************************/

#include "CQJsonModel.h"
#include <QFile>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QIcon>
#include <QFont>

CQJsonModel::CQJsonModel(QObject *parent, QList<QJsonObject> list) :
    QAbstractItemModel(parent)
{
    mRootItem = new QJsonTreeItem;
    mHeaders.append("key");
    mHeaders.append("value");
    for (int i = 0; i < list.size(); i++) {
        loadQJson(list.value(i));
    }
}

void CQJsonModel::saveSettings(int row, QUrl filename)
{
    QJsonObject data;
    data = mRootItem->getChilds()->value(i)->toJson();
    emit saveQJson(data, filename);
}

void CQJsonModel::loadSettings(int row, QUrl filename)
{
    emit requestQJson(filename);
    mRootItem->getChilds()->removeAt(row);
    int j = mRootItem->getChilds()->size() - 1;
    mRootItem->getChilds()->swap(row, j);
}
Qt::ItemFlags CQJsonModel::flags(QModelIndex& index)
{
int row;
row = index.row();
//checks if not root item or the one below
if (row == 0 || index.parent().parent().row()) {
    return Qt::ItemIsSelectable;
} else {
    return Qt::ItemIsEditable;
}
}

bool CQJsonModel::setData(QModelIndex& index, QVariant& value, int role)
{
    if (!index.isValid())
        return false;

    if (role == Qt::EditRole) {
        CQJsonTreeItem temp = backtrack(index);
        if (value.typeName() != temp.type()) {
            return false;
        }
        temp.setValue(value);
        return true;
    }
    return false;
}
CQJsonTreeItem CQJsonModel::backtrack(QModelIndex& index)
{
    if(!index.parent().parent().isValid) //one row under root
    {
       return mRootItem->getChilds().value(index.row());
    } else {
        QJsonTreeItem temp = backtrack(index.parent());
        return temp->getChilds().value(index.row());
    }
}

void loadQJson(QJsonObject data)
{
    QJsonDocument docu = QJsonDocument(data);
    loadJson(docu.toJson());
}



bool CQJsonModel::load(const QString &fileName)
{
    QFile file(fileName);
    bool success = false;
    if (file.open(QIODevice::ReadOnly)) {
        success = load(&file);
        file.close();
    }
    else success = false;

    return success;
}

bool CQJsonModel::load(QIODevice *device)
{
    return loadJson(device->readAll());
}

bool CQJsonModel::loadJson(const QByteArray &json)
{
    mDocument = QJsonDocument::fromJson(json);

    if (!mDocument.isNull())
    {

        if (mDocument.isArray()) {
            mRootItem->appendChild(QJsonTreeItem::load(QJsonValue(mDocument.array()));
        } else {
            mRootItem->appendChild(QJsonTreeItem::load(QJsonValue(mDocument.object()));
        }

        return true;
    }
    return false;
}


QVariant CQJsonModel::data(const QModelIndex &index, int role) const
{

    if (!index.isValid())
        return QVariant();


    CQJsonTreeItem *item = static_cast<CQJsonTreeItem*>(index.internalPointer());


    if ((role == Qt::DecorationRole) && (index.column() == 0)){

        return mTypeIcons.value(item->type());
    }


    if (role == Qt::DisplayRole) {

        if (index.column() == 0)
            return QString("%1").arg(item->key());

        if (index.column() == 1)
            return QString("%1").arg(item->value());
    }



    return QVariant();

}

QVariant CQJsonModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {

        return mHeaders.value(section);
    }
    else
        return QVariant();
}

QModelIndex CQJsonModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    CQJsonTreeItem *parentItem;

    if (!parent.isValid())
        parentItem = mRootItem;
    else
        parentItem = static_cast<CQJsonTreeItem*>(parent.internalPointer());

    CQJsonTreeItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    else
        return QModelIndex();
}

QModelIndex CQJsonModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    CQJsonTreeItem *childItem = static_cast<CQJsonTreeItem*>(index.internalPointer());
    CQJsonTreeItem *parentItem = childItem->parent();

    if (parentItem == mRootItem)
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

int CQJsonModel::rowCount(const QModelIndex &parent) const
{
    CQJsonTreeItem *parentItem;
    if (parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = mRootItem;
    else
        parentItem = static_cast<CQJsonTreeItem*>(parent.internalPointer());

    return parentItem->childCount();
}

int CQJsonModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent)
    return 2;
}

void CQJsonModel::setIcon(const QJsonValue::Type &type, const QIcon &icon)
{
    mTypeIcons.insert(type,icon);
}
