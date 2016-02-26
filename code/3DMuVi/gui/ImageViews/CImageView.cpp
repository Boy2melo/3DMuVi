#include "CImageView.h"
#include <QPainter>
#include <QWheelEvent>

CImageView::CImageView(QWidget *parent){
  exp = 1;
  init = false;
  currentcolor = 0;
}


//============================================================
/*!
@param event
*/
//============================================================
void CImageView::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
   if(init)
   {painter.setTransform(m_transform);}
    init = true;
    // Paint all Images from the List

    for(uint i = 0; i <  ImageOffsetList.size(); i++ ) {
        QImage& image = std::get<2> (ImageOffsetList[i]);
        QPoint offset = std::get<1> (ImageOffsetList[i]);
        painter.drawImage(offset,image);
    }

    // for all FeaureLists  Paint all Feauture points with lines
    for(uint j = 0; j < FeatureList.size(); j++){
        //Paint List
        for(uint g = 0; g < FeatureList[j].size(); g++){
            QPoint cpoint = std::get<1> (FeatureList[j][g]);
            for(uint h = g + 1; h < FeatureList[j].size(); h++){
                 QPoint thisPoint = std::get<1> (FeatureList[j][h]);
                 painter.drawLine(cpoint,thisPoint);
            }
        }
     switch (currentcolor) {
        case 0:
            painter.setPen(Qt::magenta);
            currentcolor = 1;
            break;
        case 1:
         painter.setPen(Qt::green);
         currentcolor = 2;
            break;
        case 2:
         painter.setPen(Qt::cyan);
         currentcolor = 0;
            break;
        default:
            currentcolor = 0;
            break;
        }
    }


}

//============================================================
/*!
@param event
*/
//============================================================
void CImageView::wheelEvent(QWheelEvent* event)
{
    exp = exp + (event->delta() / 120.0);
    if(exp <= -15 ){
        exp = -15;
    }
    if(exp >= 25) {
        exp  = 25;
    }

    float factor = powf(1.41, exp);
    QTransform transform;
    transform = m_transform.fromScale(factor, factor);
    m_transform = transform;
    event->accept();
    this->update();

}

//============================================================
/*!
@param event
*/
//============================================================
void CImageView::mouseMoveEvent(QMouseEvent* event)
{

}

//============================================================
/*!
@param event
*/
//============================================================
void CImageView::mousePressEvent(QMouseEvent* event)
{

}

//============================================================
/*!
@param event
*/
//============================================================
void CImageView::mouseReleaseEvent(QMouseEvent* event)
{

}

//============================================================
/*!
@param images
*/
//============================================================
void CImageView::showImages(std::vector<std::tuple<uint32_t,QImage&>> images)
{
// clear old Image List + FeaturepointList
    ImageOffsetList.clear();
    FeatureList.clear();
//calculate the grid for the Imageview (3x3 for 9 images, 4x4 for 12 images...)
int tablesize = ceil(sqrt(images.size()));
uint iIndex = 0;
//pepArray: bottem right pixel of the images layoutet at [i][j] used to calculate the offset
QPoint pepArray [tablesize][tablesize];

//build ImageOffsetList
for(int i = 0; i < tablesize; i++){
    for(int j = 0;j < tablesize; j++ ) {
        if(iIndex < images.size()){
            uint32_t id = std::get<0> (images[iIndex]);
            QImage& iRef = std::get<1> (images[iIndex]);

            //calculate offset
            int offsetx = 0;
            int offsety = 0;
            if(i != 0){
               offsetx = (pepArray[i-1][j].rx() + 20) ;
            }
            if(j != 0){
               offsety = (pepArray[i][j-1].ry() + 20) ;
            }
            QPoint Offset(offsetx,offsety);
            //update pepArray
            pepArray[i][j] = QPoint(offsetx + iRef.width(), offsety + iRef.height());

            //add to the Offset List with calculated Offset needed for Painting Images and Feauturepoints / lines
            std::tuple<uint32_t,QPoint,QImage&> tuple (id,Offset,iRef);
            ImageOffsetList.push_back(tuple);
        }
        iIndex++;
    }
}
}


//============================================================
/*!
@param positions
*/
//============================================================
void CImageView::addConnectedMarkers(std::vector<std::tuple<uint32_t,QVector2D>> positions)
{
   std::vector<std::tuple<uint32_t,QPoint>> result;
    //calculate absolute Koordinates of the Feautruepoints
 for(uint i = 0; i < positions.size(); i++)  {
     bool idFound = false;
     QPoint ap;
     QPoint p = (std::get<1>(positions[i])).toPoint();
     uint32_t fId = std::get<0>(positions[i]);
     //search Image List for ID
     for(uint j = 0; j < ImageOffsetList.size(); j++){
         uint32_t iId= std::get<0>(ImageOffsetList[j]);
        if(fId == iId){
             //add Offset to ap(result point)
             QPoint offset = std::get<1>(ImageOffsetList[j]);
             ap = p + offset;
             idFound = true;
         }
     }
     //if Id is in the Image List Add Feauturepoint to the results with absolute coordinates
     if(idFound){
        std::tuple<uint32_t,QPoint> r (fId,ap);
        result.push_back(r);
     }
 }


 FeatureList.push_back(result);
}

