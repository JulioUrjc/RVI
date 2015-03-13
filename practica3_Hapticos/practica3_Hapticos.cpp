
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/NodeCallback>

#include <chai3d/chai3d.h>

#include "ProxyAlgorithm.h"

#define M_PI 3.14159265358979323846

// main haptics loop
void updateHaptics();

// function called before exiting the application
void close();

// function to compute forces
cVector3d compute_force();

// send the given force to the haptic device
void send_force_to_device(cVector3d force);

// a world that contains all objects of the virtual environment
cWorld* world;

// a haptic device handler
cHapticDeviceHandler* handler;

// the haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// a virtual object
cMesh* object;

// Froce proxy wrapper: replace default one by our own
ProxyPointForceAlgoWrapper* proxy_wrapper;
   
// Proxy algorithm: our own
ProxyAlgorithm* proxy_algorithm;

// simulation clock
cPrecisionClock simClock;

// status of the main simulation haptics loop
bool simulationRunning = false;

// has exited haptics simulation thread
bool simulationFinished = false;

std::string obj_filename("../../data/camera/camera.3ds");
//std::string obj_filename("../../data/drill/drill.3ds");
//std::string obj_filename("../../data/face/face.3ds");
//std::string obj_filename("../../data/tooth/tooth.3ds");
//std::string obj_filename("../../data/box/box.3ds");
//std::string obj_filename("../../data/gear/gear.3ds");
//std::string obj_filename("../../data/gear/axis.3ds");
//std::string obj_filename("../../data/pool/ball.3ds");

// friction coeficient
double friction_coeficient = 0.3;

// posiions of proxy and probe for visualization
cVector3d visual_proxy_pos(0,0,0);
cVector3d visual_probe_pos(0,0,0);

// OSG nodes for visualization
osg::PositionAttitudeTransform* proxySphereXForm;
osg::PositionAttitudeTransform* deviceSphereXForm;
osg::PositionAttitudeTransform* modelXForm;
double obj_rot = 0.;
double obj_rot_step = 0.01;

// keyboard handler to be able to rotate the object
class KeyboardEventHandler : public osgGA::GUIEventHandler
{
   public:
      virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
      {
         switch(ea.getEventType())
         {
            case(osgGA::GUIEventAdapter::KEYDOWN):
               {
                  switch(ea.getKey())
                  {
                  case osgGA::GUIEventAdapter::KEY_Left:
                     obj_rot -= obj_rot_step;
                     return false;
                     break;
                  case osgGA::GUIEventAdapter::KEY_Right:
                     obj_rot += obj_rot_step;
                     return false;
                     break;
                  default:
                     return false;
                  } 
               }
            default:
               return false;
         }
      }

      virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
};

int main(int , char **)
{
   // ******** HAPTIC DEVICE ********* //

    // create a new world.
    world = new cWorld();
    
    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);
    
   // retrieve information about the current haptic device
    cHapticDeviceInfo info;
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }

    // create a 3D tool and add it to the world
    tool = new cGeneric3dofPointer(world);

    // replace proxy algorithm by ours
    delete tool->m_proxyPointForceModel;
    proxy_wrapper = new ProxyPointForceAlgoWrapper();
    proxy_algorithm = new ProxyAlgorithm(proxy_wrapper);
    tool->m_proxyPointForceModel = proxy_wrapper;
    proxy_wrapper->initialize(world, cVector3d(0,0,0));

    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

   // hide the device sphere. only show proxy.
    tool->m_deviceSphere->setShowEnabled(false);

    // set the physical radius of the proxy. for performance reasons, it is
    // sometimes useful to set this value to zero when dealing with
    // complex objects.
    double proxyRadius = 0.01;
    tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);

    // inform the proxy algorithm to only check front sides of triangles
    tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate or translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->m_proxyPointForceModel->m_useDynamicProxy = true;
    tool->m_proxyPointForceModel->m_useFriction = true;


    // ************* HAPTIC SCENE ************** //

    // create a virtual mesh
    object = new cMesh(world);

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setPos(0.0, 0.0, 0.0);
    
    // load an object file
    bool fileload = object->loadFromFile(obj_filename);
    if (!fileload)
    {
        printf("Error in Haptics - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // compute a bounding box
    object->computeBoundaryBox(true);

    // get dimensions of object
    double size = cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();

   // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(size/2.);
    //tool->setWorkspaceRadius(size/4.);

    // define a radius for the tool
    tool->setRadius(0.01);
    
    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;

    // compute collision detection algorithm
    object->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

    // define a default stiffness for the object
    object->setStiffness(stiffnessMax, true);

    // define some haptic friction properties
    object->setFriction(friction_coeficient, friction_coeficient, true);



    // ************** OpenSceneGraph ************//
    
    // construct the viewer.
    osgViewer::Viewer viewer;

      bool use_fullscreen = false;

      if (use_fullscreen)
      {
      // set window in full screen
      viewer.setUpViewOnSingleScreen(0);
      }
      else
      {
      // set window in widonwed mode
      viewer.setUpViewInWindow(100, 100, 800, 600);
      }

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(obj_filename);
    if (!model)
    {
        printf("Error in OSG - 3D Model failed to load correctly.\n");
        //close();
        //return (-1);
    }

   modelXForm = new osg::PositionAttitudeTransform();
   modelXForm->setDataVariance( osg::Object::DYNAMIC );
   modelXForm->addChild(model);

   osg::Group* rootnode = new osg::Group;
   rootnode->addChild(modelXForm);

   // Proxy sphere
   osg::Sphere* proxySphere = new osg::Sphere( osg::Vec3(0,0,0),size/200.);
   osg::ShapeDrawable* proxySphereDrawable = new osg::ShapeDrawable(proxySphere);

   osg::Geode* proxySphereGeode = new osg::Geode();
   proxySphereGeode->addDrawable(proxySphereDrawable);

   proxySphereXForm = new osg::PositionAttitudeTransform();
   proxySphereXForm->setDataVariance( osg::Object::DYNAMIC );
   proxySphereXForm->addChild(proxySphereGeode);

   rootnode->addChild(proxySphereXForm);

   // Device sphere
   osg::Sphere* deviceSphere = new osg::Sphere( osg::Vec3(0,0,0),size/200.);
   osg::ShapeDrawable* deviceSphereDrawable = new osg::ShapeDrawable(deviceSphere);
   deviceSphereDrawable->setColor(osg::Vec4d(1.,0.,0.,1.));

   osg::Geode* deviceSphereGeode = new osg::Geode();
   deviceSphereGeode->addDrawable(deviceSphereDrawable);

   deviceSphereXForm = new osg::PositionAttitudeTransform();
   deviceSphereXForm->setDataVariance( osg::Object::DYNAMIC );
   deviceSphereXForm->addChild(deviceSphereGeode);

   rootnode->addChild(deviceSphereXForm);

   viewer.setSceneData(rootnode);

    // ******* RUN ****** //

    viewer.addEventHandler(new KeyboardEventHandler()); 

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    // run
    viewer.realize();
    while(!viewer.done() )
    {
       // update node positions in OpenSceneGraph
        modelXForm->setAttitude(osg::Quat(obj_rot, osg::Vec3d(0.0, 0.0, 1.0)));
        proxySphereXForm->setPosition(osg::Vec3d(visual_proxy_pos[0], visual_proxy_pos[1], visual_proxy_pos[2]));
        deviceSphereXForm->setPosition(osg::Vec3d(visual_probe_pos[0], visual_probe_pos[1], visual_probe_pos[2]));

        // setup camera
        viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(size*2, 0, 0), osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, 1));
        viewer.frame();
    }

   // close everything
   close();

   return 0;

}


// the haptic loop, running in its own thread
void updateHaptics()
{
    // reset clock
    simClock.reset();
    
    // main haptic simulation loop
    while(simulationRunning)
    {
       // update object orientation (from user input with keyboard)
       cMatrix3d new_rot_mat;
       new_rot_mat.set(cVector3d(0,0,1), obj_rot);   
       object->setRot(new_rot_mat);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);
        
         // update position and orientation of probe
        tool->updatePose();

       
        // compute interaction forces
        //tool->computeInteractionForces();
        // send forces to device
        //tool->applyForces();

        // compute interaction forces
        cVector3d force = compute_force();
        // send the forces to the device
        send_force_to_device(force);

		// retrieve probe and proxy positions for visual display
        visual_proxy_pos = tool->getProxyGlobalPos();
        visual_probe_pos = tool->getDeviceGlobalPos();

        // stop the simulation clock
        simClock.stop();

        // read the time increment in seconds
        double timeInterval = simClock.getCurrentTimeSeconds();

        
    }

    // exit haptics thread
    simulationFinished = true;
}

cVector3d compute_force()
{
    // Disable friction computation in proxy algorithm, since we are going to do it ourselves
   tool->m_proxyPointForceModel->m_useFriction = false;

   // get probe position
   cVector3d probe_pos = tool->getDeviceGlobalPos();
   
   // compute new proxy position given the probe position
   proxy_algorithm->computeProxyPosition(probe_pos);

   cVector3d force;
   force.zero();

   
   cVector3d x = probe_pos - proxy_algorithm->getProxyContactPosition();
   force = -proxy_algorithm->getContactObjectStiffness() * x;


   double fn_e = force.dot(proxy_algorithm->getContactNormal(0));
   cVector3d fn = fn_e * proxy_algorithm->getContactNormal(0);
   cVector3d ft = force - fn;

   if (ft.length() <= fn.length() * friction_coeficient) {
	   // Fricción estática, el proxy no se mueve de su posicion de colisión
	   proxy_algorithm->setProxyPosition(proxy_algorithm->getProxyContactPosition());
   } else {
	   // Fricción dinámica
	   ft.normalize();
	   //fn.length();
	   //float module = sqrt(pow(fn.x,2)+pow(fn.y,2)+pow(fn.z,2));
	   force = fn.length() * friction_coeficient * ft + fn;
	   //cVector3d position = (proxy_algorithm->getProxyContactPosition() + proxy_algorithm->getProxyPosition()) / 2;
	   cVector3d position = force/friction_coeficient + probe_pos;
	   proxy_algorithm->setProxyPosition(position);
   }
   
   return force;

}

void send_force_to_device(cVector3d force)
{
   // convert force into device local coordinates
   cVector3d force_local;
   cMatrix3d tRot = tool->getGlobalRot();
   tRot.mulr(force, force_local);

   hapticDevice->setForce(force_local);

}


void close()
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}
