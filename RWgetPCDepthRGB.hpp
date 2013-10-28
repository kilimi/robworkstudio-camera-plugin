#ifndef RWGETPCDEPTHRGB_HPP
#define RWGETPCDEPTHRGB_HPP


#include <rws/RobWorkStudioPlugin.hpp>
#include <string.h>

#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <RobWorkStudio.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rwlibs/simulation/SimulatedKinnect.hpp>
#include <opencv2/opencv.hpp>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rwlibs::simulation;
using namespace rw::sensor;

using namespace cv;
using namespace std;
#include "ui_RWgetPCDepthRGB.h"

using namespace rw::kinematics;

class RWgetPCDepthRGB: public rws::RobWorkStudioPlugin, private Ui::LuluWinz
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    RWgetPCDepthRGB();
    virtual ~RWgetPCDepthRGB();

    // functions inherited from RobworkStudioPlugin, are typically used but can be optional
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();


private slots:
    void sick1Event();
    void sick2Event();
    void showPointCloudEvent();
    void showDepthEvent();
    void showRGBEvent();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    //QPushButton* _btn0,*_btn1;
    //QCheckBox  _top, _bottom;

    QPushButton *pcdButtons[7];
    QPushButton *depthButtons[6];
    QPushButton *rgbButtons[6];
    pcl::PointCloud<pcl::PointXYZ> createAndSavePCD(Frame *, std::string, rw::math::Transform3D<double>, std::string, int);
    void saveDepthMap(pcl::PointCloud<pcl::PointXYZ> , string );
    void saveRgbImage(Frame*, string );

};

#endif /*RWGETPCDEPTHRGB_HPP*/
