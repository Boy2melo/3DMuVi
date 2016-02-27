#ifndef CIMAGEVIEW_H
#define CIMAGEVIEW_H

#include <QVector2D>
#include <vector>
#include <tuple>

#include <QWidget>

#include <gui/IGuiDataView.h>

class CImageView : public QWidget, public IGuiDataView
{
    Q_OBJECT
//TODO Write Doku change variables to private

protected: 
  void paintEvent(QPaintEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void showImages(std::vector<std::tuple<uint32_t,QImage&>> images);
  void addConnectedMarkers(std::vector<std::tuple<uint32_t,QVector2D>> positions);
  float exp;
  bool  init;
  int currentcolor;
  QTransform m_transform;
  std::vector<std::tuple<uint32_t,QPoint,QImage>> ImageOffsetList;
  std::vector<std::vector<std::tuple<uint32_t,QPoint>>> FeatureList;
public:
   CImageView(QWidget *parent = 0);
};

#endif // CIMAGEVIEW_H
