
#include "lulu_winz.hpp"

#include <string.h>

#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <RobWorkStudio.hpp>
#include <rw/geometry/PointCloud.hpp>
#include "ui_LuluWinz.h"
#include <rwlibs/simulation/SimulatedKinnect.hpp>
#include <opencv2/opencv.hpp>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <rw/kinematics/Kinematics.hpp>
//---------------------------
#include <iostream>
#include <map>
#include <utility>


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rwlibs::simulation;
using namespace rw::sensor;
using namespace rw::kinematics;


using namespace cv;
using namespace std;

GLFrameGrabber25D::Ptr _fgrabber;
GLFrameGrabber::Ptr _fgrabber2D;


LuluWinz::LuluWinz() : RobWorkStudioPlugin("LuluWinz", QIcon(":/pa_icon.png"))
{
    setupUi(this);
    connect(myButton, SIGNAL(clicked()), this, SLOT(sick1Event()));
    connect(run, SIGNAL(clicked()), this, SLOT(sick2Event()));
}

LuluWinz::~LuluWinz(){ /* deallocate used memory */ }

void LuluWinz::open(WorkCell* workcell)
{ 
    //rw::kinematics::MovableFrame* _frameGrabberFrame = new rw::kinematics::MovableFrame("StereoCamTop3");
    //workcell->addFrame(_frameGrabberFrame);

    //State state = workcell->getDefaultState();
    //_frameGrabberFrame->setTransform(something, state);

    // create GLFrameGrabber
    _fgrabber = ownedPtr( new GLFrameGrabber25D(640, 480, 45, 0.1) );
    _fgrabber->init( getRobWorkStudio()->getView()->getSceneViewer() );


    _fgrabber2D = ownedPtr( new GLFrameGrabber(640, 480, 45, 0.1) );
    _fgrabber2D->init( getRobWorkStudio()->getView()->getSceneViewer() );


    //getRobWorkStudio()->setState(state);
}
void LuluWinz::close() { /* do something when the workcell is closed */}

void LuluWinz::initialize() {
    /* do something when plugin is initialized */
    getRobWorkStudio()->stateChangedEvent().add(
                boost::bind(&LuluWinz::stateChangedListener, this, _1), this);
}

void LuluWinz::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

void LuluWinz::sick1Event() {
    QObject *obj = sender();
    getRobWorkStudio() -> openFile("/home/lulu/Desktop/IntellActScene/IntellActScene.xml");
    log().info() << "Loading scene\n";
}

pcl::PointCloud<pcl::PointXYZ> LuluWinz::createAndSavePCD(Frame *camFrame, std::string name, rw::math::Transform3D<double> transform)
{
    _fgrabber->grab(camFrame , getRobWorkStudio()->getState() );
    const Image25D& img = _fgrabber->getImage();

    PointCloud pcloud(img.getWidth(),img.getHeight());
    pcloud.getData() = img.getData();



    //delete the plane
    for(int i=0;i<pcloud.getData().size();i++){
        if(MetricUtil::norm2(pcloud.getData()[i])> 2){ //6
            pcloud.getData()[i] = Vector3D<float>(0,0,0);
        }
    }

    //from double to floats....
    Transform3D<float> newT;
    newT(0,0) = transform(0,0); newT(0,1) = transform(0,1); newT(0,2) = transform(0,2); newT(0,3) = transform(0,3);
    newT(1,0) = transform(1,0); newT(1,1) = transform(1,1); newT(1,2) = transform(1,2); newT(1,3) = transform(1,3);
    newT(2,0) = transform(2,0); newT(2,1) = transform(2,1); newT(2,2) = transform(2,2); newT(2,3) = transform(2,3);

    //transform to world frame
    for (int i=0;i<pcloud.getData().size();i++)
    {

        pcloud.getData()[i] = newT * pcloud.getData()[i];
        //        pcloud.getData()[i](0) = static_cast<float> (transform (0, 0) * pcloud.getData()[i](0) +
        //                                                     transform (0, 1) * pcloud.getData()[i](1) +
        //                                                     transform (0, 2) * pcloud.getData()[i](2) + transform (0, 3));

        //        pcloud.getData()[i](1) = static_cast<float> (transform (1, 0) * pcloud.getData()[i](0) +
        //                                                     transform (1, 1) * pcloud.getData()[i](1) +
        //                                                     transform (1, 2) * pcloud.getData()[i](2) + transform (1, 3));

        //        pcloud.getData()[i](2) = static_cast<float> (transform (2, 0) * pcloud.getData()[i](0) +
        //                                                     transform (2, 1) * pcloud.getData()[i](1) +
        //                                                     transform (2, 2) * pcloud.getData()[i](2) + transform (2, 3));
    }

    //from PointCloud to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud(640, 480);

    int i = 0;
    for(size_t r = 0; r < cloud.height; ++r)
    {
        for(size_t c = 0; c < cloud.width; ++c)
        {
            cloud(c,r).x = pcloud.getData()[i](0);
            cloud(c,r).y = pcloud.getData()[i](1);
            cloud(c,r).z = pcloud.getData()[i++](2);
        }
    }

    pcl::io::savePCDFileASCII(name,cloud); //save point cloud

    return cloud;
}


void LuluWinz::saveDepthMap(pcl::PointCloud<pcl::PointXYZ> cloud, string name)
{
    Mat_<ushort> depthm(cloud.height, cloud.width);
    for(size_t r = 0; r < depthm.rows; ++r)
    {
        for(size_t c = 0; c < depthm.cols; ++c)
        {
            depthm[r][c] = (ushort)(cloud(c,r).z * (-1000));
        }
    }

    imwrite(name, depthm);

    log().info() << "Saved point cloud: " << name <<"\n";
}
void LuluWinz::saveRgbImage(Frame *camFrame, string name)
{
    _fgrabber2D->grab(camFrame , getRobWorkStudio()->getState() );
    const Image& img2D = _fgrabber2D->getImage();
    img2D.saveAsPPM(name);
    log().info() << "Image saved: " << name <<"\n";
}

void LuluWinz::sick2Event() 
{
    QObject *obj = sender();

    map<string, QCheckBox*> checkBoxMap;
    checkBoxMap["top"] = _top;
    checkBoxMap["bottom"] = _bottom;

    for( map<string, QCheckBox*>::iterator ii=checkBoxMap.begin(); ii!=checkBoxMap.end(); ++ii)
    {
       //cout << (*ii).first << ": " << (*ii).second << endl;

    }

    Frame *camFrameTop = getRobWorkStudio()->getWorkCell()->findFrame("top.LeftVisu");
    Frame *camFrameBottom = getRobWorkStudio()->getWorkCell()->findFrame("bottom.LeftVisu");
    Frame *camFrameRight = getRobWorkStudio()->getWorkCell()->findFrame("right.LeftVisu");
    Frame *camFrameLeft = getRobWorkStudio()->getWorkCell()->findFrame("left.LeftVisu");
    Frame *camFrameRight2 = getRobWorkStudio()->getWorkCell()->findFrame("right2.LeftVisu");
    Frame *camFrameLeft2 = getRobWorkStudio()->getWorkCell()->findFrame("left2.LeftVisu");

    pcl::PointCloud<pcl::PointXYZ> top;
    pcl::PointCloud<pcl::PointXYZ> bottom;
    pcl::PointCloud<pcl::PointXYZ> right;
    pcl::PointCloud<pcl::PointXYZ> left;
    pcl::PointCloud<pcl::PointXYZ> right2;
    pcl::PointCloud<pcl::PointXYZ> left2;

    pcl::PointCloud<pcl::PointXYZ> fullPontCloud;

    rw::math::Transform3D<double> wTcu = Kinematics::worldTframe(camFrameTop, getRobWorkStudio()->getState());
    top = createAndSavePCD(camFrameTop, std::string("top.pcd"), wTcu);
    log().info() << "Loaded top " <<"\n";

    rw::math::Transform3D<double> wTcd =  Kinematics::worldTframe(camFrameBottom, getRobWorkStudio()->getState());
    bottom = createAndSavePCD(camFrameBottom, std::string("bottom.pcd"), wTcd);
    log().info() << "Loaded bottom " <<"\n";

    rw::math::Transform3D<double> wTcr =  Kinematics::worldTframe(camFrameRight, getRobWorkStudio()->getState());
    right = createAndSavePCD(camFrameRight, std::string("right.pcd"), wTcr);
    log().info() << "Loaded right " <<"\n";

    rw::math::Transform3D<double> wTcl =  Kinematics::worldTframe(camFrameLeft, getRobWorkStudio()->getState());
    left = createAndSavePCD(camFrameLeft, std::string("left.pcd"), wTcl);
    log().info() << "Loaded left " <<"\n";

    rw::math::Transform3D<double> wTcr2 =  Kinematics::worldTframe(camFrameRight2, getRobWorkStudio()->getState());
    right2 = createAndSavePCD(camFrameRight2, std::string("right2.pcd"), wTcr2);
    log().info() << "Loaded right2 " <<"\n";

    rw::math::Transform3D<double> wTcl2 =  Kinematics::worldTframe(camFrameLeft2, getRobWorkStudio()->getState());
    left2 = createAndSavePCD(camFrameLeft2, std::string("left2.pcd"), wTcl2);
    log().info() << "Loaded left2 " <<"\n";

    top += bottom;
    right += left;
    right2+= left2;
    top += right;
    top += right2;
    fullPontCloud = top;

    pcl::io::savePCDFileASCII("combined.pcd", fullPontCloud);
    log().info() << "Finished " <<"\n";
}

Q_EXPORT_PLUGIN2(LuluWinz, LuluWinz);
