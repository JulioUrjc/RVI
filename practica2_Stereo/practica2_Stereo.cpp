
#include "tracker.h"

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/LineWidth>
#include <osg/BlendFunc>

#include <string>
#include <iostream>

using namespace osg;


void setProjectionMatrixDIO(osg::Camera* camera, osg::Vec3d pa, osg::Vec3d pb, osg::Vec3d pc, osg::Vec3d pe, double n, double f, double DIO_2);
void setProjectionMatrix(osg::Camera* camera, osg::Vec3d pa, osg::Vec3d pb, osg::Vec3d pc, osg::Vec3d pe, double n, double f);
void createCage(osg::Group* parent, osg::Vec3d lower_left, osg::Vec3d lower_right, osg::Vec3d upper_left);
void createTargets(osg::Group* parent, osg::Vec3d lower_left, osg::Vec3d lower_right, osg::Vec3d upper_left);
void createTexturedSquare(osg::Geode* geode);
void createHorizontalCylinder(osg::Geode* geode, double radius, double height);

#define M_PI 3.14159265358979323846
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))


osgViewer::Viewer viewer;
bool fullScreen;
bool offaxis=false;
bool toein=true;

double width, height;

float dio = 0.03;


class KeyboardEventHandler : public osgGA::GUIEventHandler
    {
    public:
       virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
       virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
    };

    bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
     {
       switch(ea.getEventType())
       {
       case(osgGA::GUIEventAdapter::KEYDOWN):
          {
             switch(ea.getKey())
             {
             case 'h':
				 cout << "Hola" << endl;
				 return false;
				 break;
			case '+':
				 cout << "aumento dio" <<endl;
				 dio += 0.001;
				 break;
			case '-':
				 cout << "disminuir dio" <<endl;
				 dio -= 0.001;
				 break;
			case 't':
				cout << "activamos toe-in" <<endl;
				 toein = true;
				 offaxis = false;
				 break;
			case 'o':
				cout << "activamos off-axis" <<endl;
				 offaxis = true;
				 toein = false;
				 break;
			case 'c':
				cout << "activamos side-by-side" <<endl;
				 offaxis = false;
				 toein = false;
				 break;
             default:
                return false;
             } 
          }
       default:
          return false;
       }
    }

int main(int , char **)
{
	// ******* SETUP WORLD COORDINATES ****** //

	// set screen coordinates with respect to origin
	// origin is center of screen portatil
	osg::Vec3d lower_left(-0.17, -0.0975, 0);
	osg::Vec3d lower_right(0.17, -0.0975, 0);
	osg::Vec3d upper_left(-0.17, 0.0975, 0);
	// Ordenador laboratorios
	//osg::Vec3d lower_left(-0.24, -0.135, 0);
	//osg::Vec3d lower_right(0.24, -0.135, 0);
	//osg::Vec3d upper_left(-0.24, 0.135, 0);

	width = lower_right.x()*2;
	height = upper_left.y()*2;

	// near plane
	double near_plane = 0.01;
	// far plane
	double far_plane = 3.;

	// ************** WINDOW ************ //
    fullScreen = false;
    if (fullScreen) viewer.setUpViewOnSingleScreen(0);
    else viewer.setUpViewInWindow(100, 100, 800, 600);
   
	  
	viewer.setLightingMode(osg::View::NO_LIGHT);


   // ********** Keyboard events ******* //
   KeyboardEventHandler* keh = new KeyboardEventHandler();
   viewer.addEventHandler(keh); 


   // ************** SCENE ************ //

	// create the cage
    osg::Group* cage = new osg::Group();
	createCage(cage, lower_left, lower_right, upper_left);

	// create the targets
	osg::Group* targets = new osg::Group();
	createTargets(targets, lower_left, lower_right, upper_left);

	// group them
	osg::Group* scene = new osg::Group();
	scene->addChild(cage);
	scene->addChild(targets);


   // ************** CAMERAS ************ //
	
	// left camera is a new camera
	osg::Camera* left_camera = new osg::Camera;
	// right camera is the default camera
	osg::Camera* right_camera = viewer.getCamera();

    // set up the background color
    left_camera->setClearColor(osg::Vec4(1.f,1.0f,1.0f,1.0f));

	// the left camera clears both buffers
    left_camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// the right camera clears only the zbuffer (avoids erasing what was rendered by the left camera)
	right_camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    // left camera has same viewport as right camera
	left_camera->setViewport(right_camera->getViewport());

    // set the left camera to render before the right (main) camera.
	left_camera->setRenderOrder(osg::Camera::PRE_RENDER);
	left_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	// scene graphs to be rendered by left and right cameras (left and right eyes)
	osg::Group* group_left = new osg::Group;
	osg::Group* group_right = new osg::Group;

	// add scene to both scenegraphs
	group_left->addChild(scene);
	group_right->addChild(scene);
	
    // add graph to be rendered by left camera
    left_camera->addChild(group_left);

	// create root node
	osg::Group* rootnode = new osg::Group;

	// add new camera to root graph
  	rootnode->addChild(left_camera);

	// add graph to be rendered by right (main) camera
	rootnode->addChild(group_right);
		
	// set root node
	viewer.setSceneData(rootnode);

	osg::StateSet * sceneStateSet_left = new osg::StateSet();
    osg::StateSet * sceneStateSet_right = new osg::StateSet();


	// ********** ANAGLYPH MASKS *********** //


	// *********** BLENDING ************** //

	osg::ColorMask* sceneColorMask_left = new osg::ColorMask;
	osg::ColorMask* sceneColorMask_right = new osg::ColorMask;
	
	sceneColorMask_left->setMask(true,false,false,0);
	sceneColorMask_right->setMask(false,true,true,0);

	sceneStateSet_left->setAttribute(sceneColorMask_left);
	sceneStateSet_right->setAttribute(sceneColorMask_right);

	osg::BlendFunc *blendFunc = new osg::BlendFunc(); 
	blendFunc->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA); 
	sceneStateSet_right->setMode(GL_BLEND, osg::StateAttribute::ON); 
	sceneStateSet_right->setAttributeAndModes(blendFunc, osg::StateAttribute::ON); 
	sceneStateSet_left->setMode(GL_BLEND, osg::StateAttribute::ON); 
	sceneStateSet_left->setAttributeAndModes(blendFunc, osg::StateAttribute::ON); 

	group_left->setStateSet(sceneStateSet_left);
	group_right->setStateSet(sceneStateSet_right);


	// ********* TRACKING *********** //

	// create and start tracker
	Tracker tracker;
	tracker.start();
	
	osg::Vec3d position(0,0,0.5);


	// ******* RUN ****** //

	// run viewer
	viewer.realize();

	
	while (!viewer.done()){
		osg::Vec3d head_position(position);
		tracker.getPosition(position);

		//osg::Vec3d head_position(position);

		
		// compute eye positions
		osg::Vec3d left_eye = head_position; 
		osg::Vec3d right_eye = head_position; 

		left_eye.x() -= dio/2;
		right_eye.x() += dio/2;
		
		if (toein)
		{
			//Toe-in
			float alfa = atan2(head_position[2],upper_left[1]);
			float aspect = width/height;

			left_camera->setProjectionMatrixAsPerspective(alfa*2,aspect,near_plane,far_plane);
			//right_camera->setProjectionMatrixAsPerspective(alfa*2,aspect,near_plane,far_plane);q

			left_camera->setViewMatrixAsLookAt (left_eye, osg::Vec3f(0, 0, 0), osg::Vec3f(0, 1, 0)); 
			right_camera->setViewMatrixAsLookAt(right_eye, osg::Vec3f(0, 0, 0), osg::Vec3f(0, 1, 0));
			left_camera->setProjectionMatrix(right_camera->getProjectionMatrix());
		}
		if (offaxis){
			//Off-axis
			double left = near_plane * - (width/2 - dio/2) / head_position.z();
			double right = near_plane * (width/2 + dio/2) / head_position.z();
			double top = near_plane * (height/2) / head_position.z();
			double bottom = - near_plane * (height/2) / head_position.z() ;

			//left_camera->setViewMatrixAsLookAt (left_eye, osg::Vec3f(-dio/2, 0, 0), osg::Vec3f(0, 1, 0));
			//right_camera->setViewMatrixAsLookAt(right_eye, osg::Vec3f(dio/2, 0, 0), osg::Vec3f(0, 1, 0));

			left_camera->setViewMatrix (osg::Matrixd::translate(-left_eye));
			right_camera->setViewMatrix(osg::Matrixd::translate(-right_eye));

			//std::cout << head_position.z() << " " <<  lower_right[0] << " " << dio << " " << left << " " << right << " " << bottom << " " << top << std::endl;

			left_camera->setProjectionMatrixAsFrustum(left,right,bottom,top,near_plane,far_plane);
		
			left = near_plane * - (width/2 + dio/2) / head_position.z();
			right = near_plane * (width/2 - dio/2) / head_position.z();
			top = near_plane * (height/2) / head_position.z();
			bottom = - near_plane * (height/2) / head_position.z() ;		
			
			right_camera->setProjectionMatrixAsFrustum(left,right,bottom,top,near_plane,far_plane);
		
		}
		
		// render frame
		viewer.frame();

		// sleep for a while
		OpenThreads::Thread::microSleep(10000);

	}


	// wait for thread to shut down
	OpenThreads::Thread::microSleep(1000000);

	return 0;

}




void createCage(osg::Group* parent, osg::Vec3d lower_left, osg::Vec3d lower_right, osg::Vec3d upper_left)
{

    double num_slices = 4;
    
    std::vector<osg::Vec3d> positions;
    
    // LEFT
    osg::Vec3d start = upper_left;
    double step = (upper_left[1] - lower_left[1])/num_slices;
    for (unsigned int i=0; i<(num_slices+1); i++)
    {
        positions.push_back(start - osg::Vec3d(0., step*i, 0.));
    }

    // RIGHT
    start = lower_right + osg::Vec3d(0.,upper_left[1]-lower_left[1],0.);
    step = (upper_left[1] - lower_left[1])/num_slices;
    for (unsigned int i=0; i<(num_slices+1); i++)
    {
        positions.push_back(start - osg::Vec3d(0., step*i, 0.));
    }

    // UP
    start = upper_left;
    step = (lower_right[0] - lower_left[0])/num_slices;
    for (unsigned int i=0; i<(num_slices+1); i++)
    {
        positions.push_back(start + osg::Vec3d(step*i, 0., 0.));
    }

    // DOWN
    start = lower_left;
    step = (lower_right[0] - lower_left[0])/num_slices;
    for (unsigned int i=0; i<(num_slices+1); i++)
    {
        positions.push_back(start + osg::Vec3d(step*i, 0., 0.));
    }

    // add targets
    osg::Geode* cyl_geode = new osg::Geode;
    double radius = 0.0025;
    double height = 1.;
    createHorizontalCylinder(cyl_geode, radius, height);
    for (unsigned int i=0; i<positions.size(); i++)
    {
        osg::PositionAttitudeTransform* targetXForm;
        targetXForm = new osg::PositionAttitudeTransform();
        targetXForm->setPosition(positions[i] + osg::Vec3d(0.,0.,-height/2.));
        targetXForm->setDataVariance( osg::Object::STATIC );
        targetXForm->addChild(cyl_geode);
        parent->addChild(targetXForm);
    }
    
}


void ccreateCage(osg::Group* parent, osg::Vec3d lower_left, osg::Vec3d lower_right, osg::Vec3d upper_left)
{

	osg::Geode* cage_geode = new osg::Geode();

	double num_slices = 4;
	double depth = -0.5;

	osg::ref_ptr<osg::Geometry> beam( new osg::Geometry);
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

	// LEFT
	osg::Vec3d start = upper_left;
	osg::Vec3d end = start + osg::Vec3d(0.,0.,depth);
	double step = (upper_left[1] - lower_left[1])/num_slices;
	double i1;
	for (i1=0; i1<(num_slices+1); i1++)
	{
		points->push_back(start - osg::Vec3d(0., step*i1, 0.));
		points->push_back(end - osg::Vec3d(0., step*i1, 0.));
	}

	// RIGHT
	start = lower_right + osg::Vec3d(0.,upper_left[1]-lower_left[1],0.);
	end = start + osg::Vec3d(0.,0.,depth);
	step = (upper_left[1] - lower_left[1])/num_slices;
	double i2;
	for (i2=0; i2<(num_slices+1); i2++)
	{
		points->push_back(start - osg::Vec3d(0., step*i2, 0.));
		points->push_back(end - osg::Vec3d(0., step*i2, 0.));
	}

	// UP
	start = upper_left;
	end = start + osg::Vec3d(0.,0.,depth);
	step = (lower_right[0] - lower_left[0])/num_slices;
	double i3;
	for (i3=0; i3<(num_slices+1); i3++)
	{
		points->push_back(start + osg::Vec3d(step*i3, 0., 0.));
		points->push_back(end + osg::Vec3d(step*i3, 0., 0.));
	}

	// DOWN
	start = lower_left;
	end = start + osg::Vec3d(0.,0.,depth);
	step = (lower_right[0] - lower_left[0])/num_slices;
	double i4;
	for (i4=0; i4<(num_slices+1); i4++)
	{
		points->push_back(start + osg::Vec3d(step*i4, 0., 0.));
		points->push_back(end + osg::Vec3d(step*i4, 0., 0.));
	}

	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4d(1.0,1.0,1.0,1.0));
	beam->setVertexArray(points.get());
	beam->setColorArray(color.get());
	beam->setColorBinding(osg::Geometry::BIND_OVERALL);
	beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,(i1+i2+i3+i4)*2));

	// thicker lines
	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(3.0f);
	cage_geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

	// antialiasing
	cage_geode->getOrCreateStateSet()->setMode( GL_LINE_SMOOTH, osg::StateAttribute::ON );
//	cage_geode->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );

	cage_geode->addDrawable(beam);

	parent->addChild(cage_geode);

}

void createTargets(osg::Group* parent, osg::Vec3d lower_left, osg::Vec3d lower_right, osg::Vec3d upper_left)
{
	// target positions
	std::vector<osg::Vec3d> target_positions;
	target_positions.push_back(osg::Vec3d(0.07,0.07,-0.15)); //p
	target_positions.push_back(osg::Vec3d(-0.15,0.1,-0.12));
	target_positions.push_back(osg::Vec3d(0.05,-0.07,0.));
	target_positions.push_back(osg::Vec3d(0.0,-0.12,-0.5));
	target_positions.push_back(osg::Vec3d(0.20,-0.03,-0.08));
	target_positions.push_back(osg::Vec3d(-0.03,0.10,-0.25));
	target_positions.push_back(osg::Vec3d(-0.22,-0.08,-0.7));
	target_positions.push_back(osg::Vec3d(-0.04,-0.03,0.20));
	target_positions.push_back(osg::Vec3d(-0.12,-0.05,-0.2)); //p


	osg::ref_ptr<osg::Geometry> target_lines( new osg::Geometry);
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

	osg::Vec3d depth(0.,0.,1);

	osg::Geode* cyl_geode = new osg::Geode;
    double radius = 0.005;
    double height = 1.;
    createHorizontalCylinder(cyl_geode, radius, height);
    for (unsigned int i=0; i<target_positions.size(); i++)
    {
        osg::PositionAttitudeTransform* targetXForm;
        targetXForm = new osg::PositionAttitudeTransform();
        targetXForm->setPosition(target_positions[i] + osg::Vec3d(0.,0.,-height/2.-0.001));
        targetXForm->setDataVariance( osg::Object::STATIC );
        targetXForm->addChild(cyl_geode);
        parent->addChild(targetXForm);
    }



	/*
	// add lines
	for (unsigned int i=0; i<target_positions.size(); i++)
	{
		points->push_back(target_positions[i]);
		points->push_back(target_positions[i]-depth);
	}

	// create lines geometry
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4d(1.0,1.0,1.0,1.0));
	target_lines->setVertexArray(points.get());
	target_lines->setColorArray(color.get());
	target_lines->setColorBinding(osg::Geometry::BIND_OVERALL);
	target_lines->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,target_positions.size()*2));

	osg::Geode* target_lines_geode = new osg::Geode();
	target_lines_geode->addDrawable(target_lines);

	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(3.0f);
	target_lines_geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
	target_lines_geode->getOrCreateStateSet()->setMode( GL_LINE_SMOOTH, osg::StateAttribute::ON );
	target_lines_geode->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );

	parent->addChild(target_lines_geode);
*/
	// add targets
	osg::Geode* square_geode = new osg::Geode;
	createTexturedSquare(square_geode);
	for (unsigned int i=0; i<target_positions.size(); i++)
	{
		osg::PositionAttitudeTransform* targetXForm;
		targetXForm = new osg::PositionAttitudeTransform();
		targetXForm->setPosition(target_positions[i]);
		targetXForm->setDataVariance( osg::Object::STATIC );
		targetXForm->addChild(square_geode);
		parent->addChild(targetXForm);
	}

}

// create one target
void createTexturedSquare(osg::Geode* geode)
{
	double side = 0.03;
	osg::Vec3 top_left(-side/2,side/2,0.);
	osg::Vec3 bottom_left(-side/2,-side/2,0.);
	osg::Vec3 bottom_right(side/2,-side/2,0.);
	osg::Vec3 top_right(side/2,side/2,0.);
	
	// create geometry
	osg::Geometry* geom = new osg::Geometry;
	
	osg::Vec3Array* vertices = new osg::Vec3Array(4);
	(*vertices)[0] = top_left;
	(*vertices)[1] = bottom_left;
	(*vertices)[2] = bottom_right;
	(*vertices)[3] = top_right;
	geom->setVertexArray(vertices);
	
	osg::Vec2Array* texcoords = new osg::Vec2Array(4);
	(*texcoords)[0].set(0.0f, 0.0f);
	(*texcoords)[1].set(1.0f, 0.0f);
	(*texcoords)[2].set(1.0f, 1.0f);
	(*texcoords)[3].set(0.0f, 1.0f);
	geom->setTexCoordArray(0,texcoords);
	
	osg::Vec3Array* normals = new osg::Vec3Array(1);
	(*normals)[0].set(0.0f,0.0f,0.0f);
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	
	osg::Vec4Array* colors = new osg::Vec4Array(1);
	(*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
	
	// load image
//	osg::Image* img = osgDB::readImageFile("../../data/cartadeajuste.jpg");
	osg::Image* img = osgDB::readImageFile("../../data/target-bw.png");
	if (!img)
	{
		std::cout << "Couldn't load texture." << std::endl;
		return;
	}
	
	// setup texture
	osg::Texture2D* texture = new osg::Texture2D();
	texture->setDataVariance(osg::Object::STATIC);
	texture->setImage(img);
	
	// setup state
	osg::StateSet* state = geom->getOrCreateStateSet();
	state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
	
	geode->addDrawable(geom);

}

void createHorizontalCylinder(osg::Geode* geode, double radius, double height)
{
  
    osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3d(0,0,0), radius, height);
    osg::Quat cyl_rot(osg::PI/2., osg::Vec3d(1,0,0));
//    cyl->setRotation(cyl_rot);
    osg::ShapeDrawable* cyl_drawable = new osg::ShapeDrawable(cyl);
    cyl_drawable->setColor(Vec4(0.25,0.75,0.25,1));
	cyl_drawable->setColor(Vec4(0.5,0.5,0.5,1));
	geode->addDrawable(cyl_drawable);

}
