#ifndef SPOSE_H
#define SPOSE_H


#include <QVector3D>
#include <QVector2D>
#include <QQuaternion>


struct SPose {
public:
    uint64_t cameraId;
    QVector3D translation;
    QQuaternion orientation;
    QVector2D principalPoint;
    std::vector<float> focalLength;
    std::vector<float> distortion;
};

#endif // SPOSE_H
