
#include "RWgetPCDepthRGB.hpp"

#include <string.h>

#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <RobWorkStudio.hpp>
#include <rw/geometry/PointCloud.hpp>
#include "ui_RWgetPCDepthRGB.h"
#include <rwlibs/simulation/SimulatedKinnect.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rwlibs::simulation;
using namespace rw::sensor;
using namespace rw::kinematics;


using namespace cv;
using namespace std;

#define _size_of_cameras 7

GLFrameGrabber25D::Ptr _fgrabber;
GLFrameGrabber::Ptr _fgrabber2D;



RWgetPCDepthRGB::RWgetPCDepthRGB() : RobWorkStudioPlugin("RWgetPCDepthRGB", QIcon(":/pa_icon.png"))
{
    setupUi(this);
    connect(myButton, SIGNAL(clicked()), this, SLOT(sick1Event()));
    connect(run, SIGNAL(clicked()), this, SLOT(sick2Event()));


    pcdButtons[0] = _showPointCloudtop;
    pcdButtons[1] = _showPointCloudbottom;
    pcdButtons[2] = _showPointCloudright;
    pcdButtons[3] = _showPointCloudleft;
    pcdButtons[4] = _showPointCloudright2;
    pcdButtons[5] = _showPointCloudleft2;
    pcdButtons[6] = _showPointCloudcombined;

    depthButtons[0] = _showDepthTop;
    depthButtons[1] = _showDepthBottom;
    depthButtons[2] = _showDepthRight;
    depthButtons[3] = _showDepthLeft;
    depthButtons[4] = _showDepthRight2;
    depthButtons[5] = _showDepthLeft2;

    rgbButtons[0] = _showRGBTop;
    rgbButtons[1] = _showRGBBottom;
    rgbButtons[2] = _showRGBRight;
    rgbButtons[3] = _showRGBLeft;
    rgbButtons[4] = _showRGBRight2;
    rgbButtons[5] = _showRGBLeft2;



    //pcd buttons
    for(int i = 0; i < _size_of_cameras; i++)
    {
        connect(pcdButtons[i], SIGNAL(clicked()), this, SLOT(showPointCloudEvent()));
    }
    //depth buttons
    for(int i = 0; i < _size_of_cameras-1; i++)
    {
        connect(depthButtons[i], SIGNAL(clicked()), this, SLOT(showDepthEvent()));
    }
    //rgb buttons
    for(int i = 0; i < _size_of_cameras-1; i++)
    {
        connect(rgbButtons[i], SIGNAL(clicked()), this, SLOT(showRGBEvent()));
    }

}

RWgetPCDepthRGB::~RWgetPCDepthRGB(){ /* deallocate used memory */ }

void RWgetPCDepthRGB::open(WorkCell* workcell)
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
void RWgetPCDepthRGB::close() { /* do something when the workcell is closed */}

void RWgetPCDepthRGB::initialize() {
    /* do something when plugin is initialized */
    getRobWorkStudio()->stateChangedEvent().add(
                boost::bind(&RWgetPCDepthRGB::stateChangedListener, this, _1), this);
}

void RWgetPCDepthRGB::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

void RWgetPCDepthRGB::sick1Event() {
    QObject *obj = sender();
    getRobWorkStudio() -> openFile("/home/lulu/Desktop/IntellActScene/IntellActScene.xml");
    run->setEnabled(true);
    log().info() << "Scene loaded\n";

}
void RWgetPCDepthRGB::showPointCloudEvent()
{
    QPushButton *button = (QPushButton *)sender();
    QString str;

    str.append("pcl_viewer ");
    str.append(button->objectName().remove(0, 15).toLower());
    str.append(".pcd &");

    //    log().info() << str.toLocal8Bit().data();
    system(str.toLocal8Bit().data());

}

void RWgetPCDepthRGB::showDepthEvent()
{
    QPushButton *button = (QPushButton *)sender();
    QString str;

    str.append("eog ");
    str.append(button->objectName().remove(0, 10).toLower());
    str.append(".png &");

    system(str.toLocal8Bit().data());
}

void RWgetPCDepthRGB::showRGBEvent()
{
    QPushButton *button = (QPushButton *)sender();
    QString str;

    str.append("eog ");
    str.append(button->objectName().remove(0, 8).toLower());
    str.append(".ppm &");

    system(str.toLocal8Bit().data());
}


pcl::PointCloud<pcl::PointXYZ> RWgetPCDepthRGB::createAndSavePCD(Frame *camFrame, std::string name, rw::math::Transform3D<double> transform, std::string  depth, int num)
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
    //    for (int i=0;i<pcloud.getData().size();i++)
    //    {
    //        pcloud.getData()[i] = newT * pcloud.getData()[i];
    //    }

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

    //this should be the more right point cloud, but contains extra dot..
    //    if (_savePointCloud->isChecked()) pcl::io::savePCDFileASCII(name,cloud); //save point cloud
    if (_saveDepth->isChecked())
    {
        saveDepthMap(cloud, depth);
        depthButtons[num]->setEnabled(true);
    }

    //remove dots...
    pcl::PointCloud<pcl::PointXYZ> tempCropped;
    tempCropped.reserve(cloud.size());
    for (int i=0;i<cloud.size();i++)
    {
        const pcl::PointXYZ& p = cloud[i];
        if(p.x !=0 || p.y!=0 || p.z!= 0)
            tempCropped.push_back(p);
    }

    //transform to world frame
    for (int i=0; i < tempCropped.size(); i++)
    {
        float x = tempCropped[i].x;
        float y = tempCropped[i].y;
        float z = tempCropped[i].z;
        tempCropped[i].x  = static_cast<float> (transform (0, 0) * x +
                                                transform (0, 1) * y +
                                                transform (0, 2) * z + transform (0, 3));

        tempCropped[i].y = static_cast<float> (transform (1, 0) * x +
                                               transform (1, 1) * y +
                                               transform (1, 2) * z + transform (1, 3));

        tempCropped[i].z = static_cast<float> (transform (2, 0) * x +
                                               transform (2, 1) * y +
                                               transform (2, 2) * z + transform (2, 3));


    }
    //without that dot point cloud...
    if (_savePointCloud->isChecked()) pcl::io::savePCDFileASCII(name,tempCropped); //save point cloud
    return tempCropped;
}


void RWgetPCDepthRGB::saveDepthMap(pcl::PointCloud<pcl::PointXYZ> cloud, string name)
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
void RWgetPCDepthRGB::saveRgbImage(Frame *camFrame, string name)
{
    _fgrabber2D->grab(camFrame , getRobWorkStudio()->getState() );
    const Image& img2D = _fgrabber2D->getImage();
    img2D.saveAsPPM(name);
}


void RWgetPCDepthRGB::sick2Event()
{
    QObject *obj = sender();


    //load checkboxes into array
    QCheckBox* checkBoxArray[_size_of_cameras];
    checkBoxArray[0]= _top;
    checkBoxArray[1]= _bottom;
    checkBoxArray[2]= _right;
    checkBoxArray[3]= _left;
    checkBoxArray[4]= _right2;
    checkBoxArray[5]= _left2;

    pcl::PointCloud<pcl::PointXYZ> fullPontCloud;

    for(int i = 0; i < _size_of_cameras-1; i++)
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
            log().info() <<pcd << "\n";
            log().info() <<wTc << "\n";
            pcl::PointCloud<pcl::PointXYZ> pointCloud = createAndSavePCD(camFrame, pcd, wTc, depth, i);
            if (_saveRGB->isChecked())
            {
                saveRgbImage(camFrame, rgb);
                rgbButtons[i] -> setEnabled(true);
            }

            if (_savePointCloud->isChecked())
            {
                fullPontCloud += pointCloud;
                pcdButtons[i]->setEnabled(true);
            }

        }
    }

    if (_savePointCloud)
    {
        pcl::io::savePCDFileASCII("combined.pcd", fullPontCloud);
        pcdButtons[6]->setEnabled(true);
    }
    log().info() << "Done" << "\n";
}

Q_EXPORT_PLUGIN2(RWgetPCDepthRGB, RWgetPCDepthRGB);
