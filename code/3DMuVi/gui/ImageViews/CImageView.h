#ifndef CIMAGEVIEW_H
#define CIMAGEVIEW_H

#include <vector>
#include <tuple>

#include <QWidget>

#include <gui/IGuiDataView.h>

class CImageView : public QWidget, public IGuiDataView
{
protected: 
  void paintEvent(QPaintEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void showImages(std::vector<std::tuple<uint32_t,QImage&>> images);
  void addConnectedMarkers(std::vector<std::tuple<uint32_t,QVector2D>> positions);
};

#endif // CIMAGEVIEW_H
