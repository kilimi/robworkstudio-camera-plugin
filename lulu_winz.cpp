
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
    run->setEnabled(true);
    log().info() << "Scene loaded\n";

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

    if (_savePointCloud->isChecked()) pcl::io::savePCDFileASCII(name,cloud); //save point cloud

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
}
void LuluWinz::saveRgbImage(Frame *camFrame, string name)
{
    _fgrabber2D->grab(camFrame , getRobWorkStudio()->getState() );
    const Image& img2D = _fgrabber2D->getImage();
    img2D.saveAsPPM(name);
}


void LuluWinz::sick2Event() 
{
    QObject *obj = sender();

    //load checkboxes into array
    int arraySize = 6;
    QCheckBox* checkBoxArray[arraySize];
    checkBoxArray[0]= _top;
    checkBoxArray[1]= _bottom;
    checkBoxArray[2]= _right;
    checkBoxArray[3]= _left;
    checkBoxArray[4]= _right2;
    checkBoxArray[5]= _left2;

    pcl::PointCloud<pcl::PointXYZ> fullPontCloud;

    for(int i = 0; i < arraySize; i++)
    {
        if (checkBoxArray[i]->isChecked())
        {
            char *c_str2 = checkBoxArray[i]->text().toLocal8Bit().data(); //from QString to char*
            char pcd[80], depth[80], rgb[80];

            //copy the frame name
            strcpy (pcd,c_str2);
            strcpy (depth,c_str2);
            strcpy (rgb,c_str2);

            //concatinate
            strcat (c_str2,".LeftVisu");
            strcat (pcd,".pcd");
            strcat (depth,".png");
            strcat (rgb,".ppm");

            Frame *camFrame = getRobWorkStudio()->getWorkCell()->findFrame(c_str2);
            rw::math::Transform3D<double> wTc = Kinematics::worldTframe(camFrame, getRobWorkStudio()->getState());

            pcl::PointCloud<pcl::PointXYZ> pointCloud = createAndSavePCD(camFrame, pcd, wTc);
            if (_saveRGB->isChecked()) saveRgbImage(camFrame, rgb);
            if (_saveDepth->isChecked()) saveDepthMap(pointCloud, depth);
            if (_savePointCloud->isChecked()) fullPontCloud += pointCloud;
        }
    }

    if (_savePointCloud) pcl::io::savePCDFileASCII("combined.pcd", fullPontCloud);
    log().info() << "Done" << "\n";
}

Q_EXPORT_PLUGIN2(LuluWinz, LuluWinz);
