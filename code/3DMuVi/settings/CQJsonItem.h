#ifndef CJSONITEM_H
#define CJSONITEM_H
#include <QtCore>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>
class CQJsonTreeItem {
public:
    CQJsonTreeItem(CQJsonTreeItem * parent);
    ~CQJsonTreeItem();
    void appendChild(CQJsonTreeItem * item);
    CQJsonTreeItem *child(int row);
    CQJsonTreeItem *parent();
    int childCount() const;
    int row() const;
    void setKey(const QString& key);
    void setValue(QString value);
    void setType(const QJsonValue::Type& type);
    QString key() const;
    QString value() const;
    QJsonValue::Type type() const;

    QList<CQJsonTreeItem *> getChilds();
    /*!
     * \brief toJson converts a treeitem to a jsonobject
     * \return the treeitem as QJsonObject
     */
    QJsonObject toJson();

    static CQJsonTreeItem* load(const QJsonValue& value, CQJsonTreeItem * parent);

protected:
    /*!
     * \brief toJsonValue converts the treeitem to a jsonvlaue
     * \return  the treeitem as QJsonValue
     */
    QJsonValue toJsonValue();

private:
    QString mKey;
    QString mValue;
    QJsonValue::Type mType;

    QList<CQJsonTreeItem*> mChilds;
    CQJsonTreeItem * mParent;


};

#endif // CJSONITEM_H
