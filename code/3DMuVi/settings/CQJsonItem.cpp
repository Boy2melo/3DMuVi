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

#include "CQJsonItem.h"

CQJsonTreeItem::CQJsonTreeItem(CQJsonTreeItem* parent) {

    mParent = parent;


}

CQJsonTreeItem::~CQJsonTreeItem() {
    qDeleteAll(mChilds);

}

QList<CQJsonTreeItem*>* CQJsonTreeItem::getChilds() {
    return &mChilds;
}

QJsonObject CQJsonTreeItem::toJson() {
    QJsonObject object;
    object.insert(mKey, this->toJsonValue());
    return object;
}

QJsonValue CQJsonTreeItem::toJsonValue() {
    QJsonValue value;
    switch (mType) {
        case QJsonValue::String: {
            value = QJsonValue(mValue);
            break;
        }
        case QJsonValue::Double: {
            value = QJsonValue(mValue.toDouble());
            break;
        }
        case QJsonValue::Bool: {
            bool boolvalue = false;
            if (mValue == "true") {
                boolvalue = true;
            }
            value = QJsonValue(boolvalue);
            break;
        }
        case QJsonValue::Object: {
            QJsonObject object;
            for (int i = 0; i < mChilds.size(); i++) {
                object.insert(mChilds.value(i)->key(), mChilds.value(i)->toJsonValue());
            }
            value = object;
            break;
        }
        case QJsonValue::Array: {
            QJsonArray array;
            for (int i = 0; i < mChilds.size(); i++) {
                array.append(mChilds.value(i)->toJsonValue());
            }
            break;
        }
        default:
            break;
    }
    return value;
}


void CQJsonTreeItem::appendChild(CQJsonTreeItem* item) {
    mChilds.append(item);
}

CQJsonTreeItem* CQJsonTreeItem::child(int row) {
    return mChilds.value(row);
}

CQJsonTreeItem* CQJsonTreeItem::parent() {
    return mParent;
}

int CQJsonTreeItem::childCount() const {
    return mChilds.count();
}

int CQJsonTreeItem::row() const {
    if (mParent)
        return mParent->mChilds.indexOf(const_cast<CQJsonTreeItem*>(this));

    return 0;
}

void CQJsonTreeItem::setKey(const QString& key) {
    mKey = key;
}

void CQJsonTreeItem::setValue(QString value) {
    mValue = value;
}

void CQJsonTreeItem::setType(const QJsonValue::Type& type) {
    mType = type;
}

QString CQJsonTreeItem::key() const {
    return mKey;
}

QString CQJsonTreeItem::value() const {
    return mValue;
}

QJsonValue::Type CQJsonTreeItem::type() const {
    return mType;
}

CQJsonTreeItem* CQJsonTreeItem::load(const QJsonValue& value, CQJsonTreeItem* parent) {


    CQJsonTreeItem* rootItem = new CQJsonTreeItem(parent);
    rootItem->setKey("root");

    if (value.isObject()) {

        //Get all QJsonValue childs
        foreach(QString key, value.toObject().keys()) {
            QJsonValue v = value.toObject().value(key);
            CQJsonTreeItem* child = load(v, rootItem);
            child->setKey(key);
            child->setType(v.type());
            rootItem->appendChild(child);

        }

    }

    else if (value.isArray()) {
        //Get all QJsonValue childs
        int index = 0;
        foreach(QJsonValue v, value.toArray()) {

            CQJsonTreeItem* child = load(v, rootItem);
            child->setKey(QString::number(index));
            child->setType(v.type());
            rootItem->appendChild(child);
            ++index;
        }
    }
    else {
        rootItem->setValue(value.toVariant().toString());
        rootItem->setType(value.type());
    }

    return rootItem;
}

