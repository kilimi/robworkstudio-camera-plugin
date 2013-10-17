
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


pcl::PointCloud<pcl::PointXYZ> LuluWinz::combinePointClouds(pcl::PointCloud<pcl::PointXYZ> master,  pcl::PointCloud<pcl::PointXYZ> slave)
{   
    // master += slave;
    for(size_t r = 0; r < master.height; ++r)
    {
        for(size_t c = 0; c < master.width; ++c)
        {
            if (master(c,r).z == 0)
            {
                master(c,r).x = slave(c,r).x;
                master(c,r).y = slave(c,r).y;
                master(c,r).z = slave(c,r).z *(-1);
            }
        }
    }
    return master;
}


pcl::PointCloud<pcl::PointXYZ> LuluWinz::createAndSavePCD(Frame *camFrame, std::string name, rw::math::Transform3D<double> transform)
{
    _fgrabber->grab(camFrame , getRobWorkStudio()->getState() );
    const Image25D& img = _fgrabber->getImage();

    PointCloud pcloud(img.getWidth(),img.getHeight());
    pcloud.getData() = img.getData();

    //delete the plane
    for(int i=0;i<pcloud.getData().size();i++){
        if(MetricUtil::norm2(pcloud.getData()[i])>2){ //6
            pcloud.getData()[i] = Vector3D<float>(0,0,0);
        }
    }

    //transform points

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        Vector3D<double> worldPoint = wTcu*cloud[i];
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

    //-------------
    //    if (cloud.is_dense)
    //    {
    //        log().info() << "cloud: " << name << " is dense\n";
    //    }
    //    //so point clouds are dense
    //    for (size_t i = 0; i < cloud.points.size (); ++i)
    //    {
    //        cloud[i].x = static_cast<float> (transform (0, 0)  * cloud[i].x + transform (0, 1) * cloud[i].y + transform (0, 2) * cloud[i].z + 0);
    //        cloud[i].y = static_cast<float> (transform (1, 0) * cloud[i].x + transform (1, 1) * cloud[i].y + transform (1, 2) * cloud[i].z + transform (1, 3));
    //        cloud[i].z = static_cast<float> (transform (2, 0) * cloud[i].x + transform (2, 1) * cloud[i].y + transform (2, 2) * cloud[i].z + transform (2, 3));
    //    }

    //--------------
    pcl::io::savePCDFileASCII(name,cloud); //save point cloud

    return cloud;

    //save depth map
    //    Mat_<ushort> depthm(cloud.height, cloud.width);
    //    for(size_t r = 0; r < depthm.rows; ++r)
    //    {
    //        for(size_t c = 0; c < depthm.cols; ++c)
    //        {
    //            depthm[r][c] = (ushort)(cloud(c,r).z * (-1000));
    //        }
    //    }

    //    imwrite("image25D.png", depthm);

    //    log().info() << "Saved point cloud: " << name <<"\n";
    //    //_---------------------------
    //    _fgrabber2D->grab(camFrame , getRobWorkStudio()->getState() );
    //    const Image& img2D = _fgrabber2D->getImage();
    //    img2D.saveAsPPM("image2D.ppm");
    //    log().info() << "Image saved: " << "image2D.ppm" <<"\n";
}

void LuluWinz::sick2Event() 
{
    QObject *obj = sender();

    Frame *camFrameUp = getRobWorkStudio()->getWorkCell()->findFrame("StereoCamTopDevice.LeftVisu");
    Frame *camFrameDown = getRobWorkStudio()->getWorkCell()->findFrame("StereoCamTopDevice1.LeftVisu");

    pcl::PointCloud<pcl::PointXYZ> up;
    pcl::PointCloud<pcl::PointXYZ> down;
    pcl::PointCloud<pcl::PointXYZ> fullPontCloud;



    if(camFrameUp!=NULL)
    {
        rw::math::Transform3D<double> wTcu =  Kinematics::worldTframe(camFrameUp, getRobWorkStudio()->getState());
        //worldTframe(camFrameUp, getRobWorkStudio()->getState());
        cout << wTcu << endl;
        //        Eigen::Matrix4f transform;
        //        transform <<      0.2,        0,   -0.4,     0,
        //                          -0.4,       0,   0.7991,  -0.25,
        //                          -0.8939,    0,   -0.4481,  0.5,
        //                          0,          0,   0,        1;

        up = createAndSavePCD(camFrameUp, std::string("up.pcd"), wTcu);
    }
    else
    {
        log().info() << "camFrameUp frame is null\n";
    }

    //    if(camFrameDown!=NULL)
    //    {
    //        down = createAndSavePCD(camFrameDown, std::string("down.pcd"));
    //    }
    //    else
    //    {
    //        log().info() << "camFrameDown frame is null\n";
    //    }

    //    fullPontCloud = combinePointClouds(up, down);
    //    pcl::io::savePCDFileASCII("combined.pcd",fullPontCloud);
}

Q_EXPORT_PLUGIN2(LuluWinz, LuluWinz);
