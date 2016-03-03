#include "CImageView.h"
#include <QPainter>
#include <QWheelEvent>
#include <QScrollBar>


CImageView::CImageView(QWidget *parent){
  Q_UNUSED(parent)
   // initilize
  exp = 0;
  mousepressed = false;
  init = false;
  currentcolor = 0;
  sizeX = 480;
  sizeY = 480;
  this->resize(sizeX,sizeY);
}
void CImageView::setScrollBars(QScrollBar* horizontal, QScrollBar* vertical)
{
h = horizontal;
v = vertical;
}

void CImageView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
    QPainter painter(this);
   if(init)
    {
       painter.setTransform(m_transform);
     }
    init = true;
    // Paint all Images from the List
    for(uint i = 0; i <  ImageOffsetList.size(); i++ ) {
        QImage image = std::get<2> (ImageOffsetList[i]);
        QPoint offset = std::get<1> (ImageOffsetList[i]);
        painter.drawImage(offset,image);
    }

    // for all FeaureLists  Paint all Feauture points with lines
    for(uint j = 0; j < FeatureList.size(); j++){
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


        //Paint List
        for(uint m = 0; m < FeatureList[j].size(); m++){
            QPoint cpoint = std::get<1> (FeatureList[j][m]);
            for(uint n = m + 1; n < FeatureList[j].size(); n++){
                 QPoint thisPoint = std::get<1> (FeatureList[j][n]);
                 painter.drawLine(cpoint,thisPoint);
            }
        }

    }
    //reset Color
    currentcolor = 0;
}


void CImageView::wheelEvent(QWheelEvent* event)
{

    exp = exp + (event->delta() / 120.0);
    if(exp <= -6 ){
        exp = -6;
    }
    if(exp >= 15) {
        exp  = 15;
    }
    float factor = powf(1.41f, exp);

    QTransform transform;
    transform = m_transform.fromScale(factor, factor);
    m_transform = transform;

    if(factor > 0){
    resize(sizeX*factor ,sizeY*factor);


    double val = v->value() / factor;
    double hal = h->value() / factor;

    double hPoint = val / sizeX;
    double vPoint = hal / sizeY;


    h->setMaximum(sizeX*factor);
    v->setMaximum(sizeY*factor);


    h->setValue(sizeX*factor*hPoint);
    v->setValue(sizeY*factor*vPoint);
    }
    event->accept();
    this->update();
}


void CImageView::mouseMoveEvent(QMouseEvent* event)
{
if(mousepressed){
QPoint currentpos = event->pos();
double deltax = pmousepoint.rx() - currentpos.rx();
double deltay = pmousepoint.ry() - currentpos.ry();

deltax = - deltax / 400.0 ;
deltay = - deltay / 400.0 ;

deltax = deltax  * sizeX / 1000.0;
deltay = deltay  * sizeY / 1000.0;
h->setValue(h->value() +   deltax );
v->setValue(v->value() +   deltay );
}
}


void CImageView::mousePressEvent(QMouseEvent* event)
{
mousepressed = true;
pmousepoint = event->pos();
}


void CImageView::mouseReleaseEvent(QMouseEvent* event)
{
Q_UNUSED(event)
mousepressed = false;
}

void CImageView::showImages(std::vector<std::tuple<uint32_t,QImage&>> images)
{
// clear old Image List + FeaturepointList
    ImageOffsetList.clear();
    FeatureList.clear();
//calculate the grid for the Imageview (3x3 for 9 images, 4x4 for 12 images...)
int tablesize = ceil(sqrt(images.size()));
uint iIndex = 0;

//pepArray: bottem right pixel of the images layoutet at [i][j] used to calculate the offset
std::vector<std::vector<QPoint>> pepArray;
//Initilize Full List with empty QPoints
for(int i = 0; i < tablesize; i++){
     std::vector<QPoint> tmpv;
    for(int j = 0;j < tablesize; j++ ) {
      tmpv.push_back(QPoint(0,0));
    }
    pepArray.push_back(tmpv);
}

//build ImageOffsetList
for(int i = 0; i < tablesize; i++){
    for(int j = 0;j < tablesize; j++ ) {
        if(iIndex < images.size()){
            uint32_t id = std::get<0> (images[iIndex]);
            QImage& iRef = std::get<1> (images[iIndex]);
            QImage  image = iRef.copy();
            //calculate offset
            int offsetx = 0;
            int offsety = 0;

            if(i != 0){
                for(int g = j; g >= 0; g--){
                    int temp = (pepArray[i-1][g].rx() + 20);
                    if(temp > offsetx){
                        offsetx = temp;
                  }
                }
            }
            if(j != 0){
                for(int g = i; g >= 0; g--){
                    int temp = (pepArray[g][j-1].ry() + 20);
                    if(temp > offsety){
                        offsety = temp;
                  }
            }
            }

            QPoint Offset(offsetx,offsety);

            //update pepArray
            std::vector<QPoint> tmpv = pepArray[i];
            tmpv.insert(tmpv.begin() + j,QPoint(offsetx + iRef.width(), offsety + iRef.height()));
            pepArray.insert(pepArray.begin() + i,tmpv);

            //add to the Offset List with calculated Offset needed for Painting Images and Feauturepoints / lines
            std::tuple<uint32_t,QPoint,QImage> tuple (id,Offset,image);
            ImageOffsetList.push_back(tuple);

            //check Maximum Size
            if( sizeX < pepArray[i][j].rx()){
                sizeX = pepArray[i][j].rx();
            }

            if( sizeY < pepArray[i][j].ry()){
                sizeY = pepArray[i][j].ry();
            }


        }
        iIndex++;
    }
}
//resize and fit scrollbars to current value;
this->resize(sizeX,sizeY);
h->setMaximum(sizeX);
v->setMaximum(sizeY);

this->update();
}

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
 this->update();
}

