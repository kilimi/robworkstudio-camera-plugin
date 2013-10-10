
#include "lulu_winz.hpp"

#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <RobWorkStudio.hpp>
#include <rw/geometry/PointCloud.hpp>
#include "ui_LuluWinz.h"
#include <rwlibs/simulation/SimulatedKinnect.hpp>



USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rwlibs::simulation;
using namespace rw::sensor;

GLFrameGrabber25D::Ptr _fgrabber;


LuluWinz::LuluWinz() : RobWorkStudioPlugin("LuluWinz", QIcon(":/pa_icon.png"))
{
    setupUi(this);
    connect(myButton, SIGNAL(clicked()), this, SLOT(sick1Event()));
    connect(run, SIGNAL(clicked()), this, SLOT(sick2Event()));
}

LuluWinz::~LuluWinz(){ /* deallocate used memory */ }

void LuluWinz::open(WorkCell* workcell)
{ 
  rw::kinematics::MovableFrame* _frameGrabberFrame = new rw::kinematics::MovableFrame("StereoCamTop3");
  workcell->addFrame(_frameGrabberFrame);

  State state = workcell->getDefaultState();
    //_frameGrabberFrame->setTransform(something, state);
  
   // create GLFrameGrabber
   _fgrabber = ownedPtr( new GLFrameGrabber25D(640, 480, 45, 0.1) );
   _fgrabber->init( getRobWorkStudio()->getView()->getSceneViewer() );

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

void LuluWinz::sick2Event() 
{
    QObject *obj = sender();

    Frame *camFrame = getRobWorkStudio()->getWorkCell()->findFrame("StereoCamTopDevice.LeftVisu");
    if(camFrame!=NULL)
    {
      _fgrabber->grab(camFrame , getRobWorkStudio()->getState() );
      const Image25D& img = _fgrabber->getImage();

     /* SimulatedKinnect* scanner = new SimulatedKinnect(camFrame->getName(), camFrame);
      rw::sensor::Image25D *_img25D;
      _img25D = (Image25D*)&scanner->getScan();
      Image* _img2D = (Image*)&scanner->getImage();
      _img2D->saveAsPPM("2Dimage.ppm");
      //PointCloud<pcl::PointXYZRGB>::Ptr cloud = toPointCloud(_img25D, _color, vueAngle);
      //PointCloud::savePCD(cloud, "MyPointCloudusingScanner.pcd");
*/
      PointCloud pcloud(img.getWidth(),img.getHeight());
      pcloud.getData() = img.getData();
      
      log().info() << pcloud.getData()[0] <<"\n";
      
      PointCloud::savePCD(pcloud, "MyPointCloud.pcd");
      log().info() << "image grabbed!\n";
    }
    else
    {
      log().info() << "Cam frame is null\n";
    }
}

Q_EXPORT_PLUGIN2(LuluWinz, LuluWinz);
