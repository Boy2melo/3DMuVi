QT+=widgets

INCLUDEPATH += /home/stefan/Studium/PSE/pcl/pcl/common/include/ /home/stefan/Studium/PSE/pcl/pcl/include/ /home/stefan/Studium/PSE/pcl/pcl/visualization/include/
INCLUDEPATH += /home/stefan/Studium/PSE/pcl/pcl/geometry/include/
INCLUDEPATH += /usr/include/eigen3/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Common/Core/ /home/stefan/Studium/PSE/VTK-build/Common/Core/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Common/Math/ /home/stefan/Studium/PSE/VTK-build/Common/Math/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Rendering/LOD/ /home/stefan/Studium/PSE/VTK-build/Rendering/LOD/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Filters/Modeling/ /home/stefan/Studium/PSE/VTK-build/Filters/Modeling/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Filters/General/ /home/stefan/Studium/PSE/VTK-build/Filters/General/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Filters/Core/ /home/stefan/Studium/PSE/VTK-build/Filters/Core/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Filters/Sources/ /home/stefan/Studium/PSE/VTK-build/Filters/Sources/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Rendering/Core/ /home/stefan/Studium/PSE/VTK-build/Rendering/Core/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Interaction/Style/ /home/stefan/Studium/PSE/VTK-build/Interaction/Style/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Common/DataModel/ /home/stefan/Studium/PSE/VTK-build/Common/DataModel/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Rendering/Annotation/ /home/stefan/Studium/PSE/VTK-build/Rendering/Annotation/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Rendering/FreeType/ /home/stefan/Studium/PSE/VTK-build/Rendering/FreeType/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Common/ExecutionModel/ /home/stefan/Studium/PSE/VTK-build/Common/ExecutionModel/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Filters/Geometry/ /home/stefan/Studium/PSE/VTK-build/Filters/Geometry/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/GUISupport/Qt/ /home/stefan/Studium/PSE/VTK-build/GUISupport/Qt/
INCLUDEPATH += /home/stefan/Studium/PSE/VTK-6.3.0/Rendering/OpenGL/ /home/stefan/Studium/PSE/VTK-build/Rendering/OpenGL/

LIBS += -L/home/stefan/Studium/PSE/VTK-build/lib/ -L/home/stefan/Studium/PSE/pcl/pcl/lib/


LIBS += -lboost_system -lvtkGUISupportQt-6.3 -lvtkCommonCore-6.3 -lvtkRenderingCore-6.3 -lvtkCommonDataModel-6.3 -lvtkCommonMath-6.3 -lpcl_common -lpcl_visualization

CONFIG += c++11

SOURCES+=./CPclView.cpp
HEADERS+=./CPclView.h
