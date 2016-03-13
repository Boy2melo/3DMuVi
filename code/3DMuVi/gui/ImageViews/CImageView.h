#ifndef CIMAGEVIEW_H
#define CIMAGEVIEW_H

#include <cstdint>

#include <QVector2D>
#include <vector>
#include <tuple>
#include <QWidget>
#include <gui/IGuiDataView.h>

class QScrollBar;
/*!
\brief CImageView
\author Tim Brodbeck

This class provides basic operations for zoom, scroll, paint images & lines.
*/
class CImageView : public QWidget, public IGuiDataView
{
    Q_OBJECT


protected: 
  /*!
   * \brief paintEvent
   * \param event
   * paint the Images and Lines set with showImages and addConnectedMarkers
   */
  void paintEvent(QPaintEvent* event);
  /*!
   * \brief wheelEvent
   * \param event
   * Zoom into the widget and addjust the scrollbars
   */
  void wheelEvent(QWheelEvent* event);
  /*!
   * \brief mouseMoveEvent
   * \param event
   * scrolls the Image if the mousebutton is pressed
   */
  void mouseMoveEvent(QMouseEvent* event);
  /*!
   * \brief mousePressEvent
   * \param event
   * set mousepressed = true and saves mousepos
   */
  void mousePressEvent(QMouseEvent* event);
  /*!
   * \brief mouseReleaseEvent
   * \param event
   * set mousepressed = false
   */
  void mouseReleaseEvent(QMouseEvent* event);
  /*!
   * \brief showImages
   * \param images vector<tuple<uint32_t,QImage&>>
   * rests ImageOffsetList and FeautureList
   * addImages to the ImageOffsetList, calculates Image Positions and adjust the size
   * calls update function
   */
  void showImages(std::vector<std::tuple<uint32_t,QImage&>> images);
  /*!
   * \brief addConnectedMarkers
   * \param positions
   * adds a number of Feauturepoints to the FeautureList
   * calls update function
   */
  void addConnectedMarkers(std::vector<std::tuple<uint32_t,QVector2D>> positions);
  float exp;
  bool  init;

  int currentcolor;
  int sizeX;
  int sizeY;
  bool mousepressed;
  QPoint pmousepoint;
  QTransform m_transform;
  std::vector<std::tuple<uint32_t,QPoint,QImage>> ImageOffsetList;
  std::vector<std::vector<std::tuple<uint32_t,QPoint>>> FeatureList;
  QScrollBar* h;
  QScrollBar* v;

public:
  /*!
    * \brief setScrollBars
    * \param horizontal
    * \param vertical
    * set scrollbars from parent ScrollArea
    */
   void setScrollBars(QScrollBar* horizontal, QScrollBar* vertical);
   /*!
    * \brief CImageView
    * \param parent
    * initilize classvariables
    */
   explicit CImageView(QWidget *parent = 0);
};

#endif // CIMAGEVIEW_H
