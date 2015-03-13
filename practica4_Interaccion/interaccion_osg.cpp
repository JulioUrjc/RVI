
#include "face_tracker.h"

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/NodeCallback>
#include <osg/Texture2D>
#include <osg/LightModel>
#include <osg/Material>
#include <osg/Timer>
#include <osg/PolygonMode>

#include <string>
#include <iostream>

const float MPI = 3.1415926535;

osgViewer::Viewer viewer;

void setProjectionMatrix(osg::Vec3d pa, osg::Vec3d pb, osg::Vec3d pc, osg::Vec3d pe, double n, double f);
void createGate(osg::Geode* geode, double width, double sep, double height);
void createFloor(osg::Geode* geode);
void createWorld(osg::Group* parent);

osg::Vec3d position_user;
osg::Vec3d init_pos (0.0,-1.0,0.0);
float angle_user;
osg::Timer timer;

const float media_velocity = 0.008;
const float altura = 0.1;
float velocity=media_velocity;
float positionprev=0.5; // Posicion central
const float MAXTIME = 1000.13; // Tiempo de 1 segundo

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
                  case osgGA::GUIEventAdapter::KEY_Left: // Izquierda
                     {
                        // completar aquí
						angle_user += 2 * MPI / 180;
						//angle_user %= 360;
                        return true;
                        break;
                     }
                  case osgGA::GUIEventAdapter::KEY_Right:  // Derecha
                     {
                        // completar aquí
						 angle_user -= 2 * MPI / 180;
						 //angle_user %= 360;
                        return true;
                        break;
                     }
                  case osgGA::GUIEventAdapter::KEY_Up:  // Arriba
                     {
                        // completar aquí
						position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,0.0,0.1);

                        return true;
                        break;
                     }
                  case osgGA::GUIEventAdapter::KEY_Down:  // Abajo
                     {
                        // completar aquí
						position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,0.0,-0.1);

                        return true;
                        break;
                     }

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
	// ************** OSG ************//
    osg::DisplaySettings::instance()->setStereoMode( osg::DisplaySettings::ANAGLYPHIC); // set stereo mode to Anaglyph
	osg::DisplaySettings::instance()->setEyeSeparation(0.03); // inter-ocular distance (to adjust)
	osg::DisplaySettings::instance()->setStereo(true); // activate stereo rendering

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

   viewer.setLightingMode(osg::View::NO_LIGHT);
   
   // create the world
   osg::Group* world_geom = new osg::Group();
	createWorld(world_geom);

	// world position and rotation (attitude)
	osg::PositionAttitudeTransform* world;
	world = new osg::PositionAttitudeTransform();
	world->setPosition(init_pos);
	world->setDataVariance( osg::Object::STATIC );
	world->addChild(world_geom);

   // add model to scenegraph
   osg::Group* rootnode = new osg::Group;
	rootnode->addChild(world);

   // set scenegraph
	viewer.setSceneData(rootnode);

	//********** TRACKING ***********//

	// create and start face tracker
	FaceTracker tracker;
	tracker.start();

	// ******* SETUP COORDINATES ****** //

   // fixed head position in real space
   osg::Vec3d position(0.,.0,0.3);

	osg::Vec2d face_2d_position;

	// set screen coordinates with respect to origin
	// origin is center of screen portatil
	osg::Vec3d lower_left(-0.17, -0.0975, 0);
	osg::Vec3d lower_right(0.17, -0.0975, 0);
	osg::Vec3d upper_left(-0.17, 0.0975, 0);
	// Ordenador laboratorio
	//osg::Vec3d lower_left(-0.24, -0.135, 0);
	//osg::Vec3d lower_right(0.24, -0.135, 0);
	//osg::Vec3d upper_left(-0.24, 0.135, 0);

	// near plane
	double near_plane = 0.01;
	// far plane
	double far_plane = 40.;

	// ******* RUN ****** //

    viewer.addEventHandler(new KeyboardEventHandler()); 

	// run viewer
	viewer.realize();
	
	bool avanza = false;
	bool agachado = false;
	bool saltar = false;

	while (!viewer.done()){

		// get head position
		tracker.getFace2DPosition(face_2d_position);	

		if (face_2d_position.x()<0.4){
			std::cout << "posPrev: " << positionprev << " timer: " << timer.time_m() << " avanza: " << avanza << " velocidad: " << velocity << std::endl;
			if ((positionprev==0) && (timer.time_m()>MAXTIME) ){
				//Situacion de giro
				angle_user -= 1.5 * MPI / 180;
				positionprev=0;
				avanza=false;
			}else if ((positionprev !=0 && timer.time_m()<MAXTIME) || avanza){
				//Situacion de avance
				//if (positionprev != 0) timer.setStartTick();
				if (positionprev != 0) {
					velocity = media_velocity/timer.time_s();
					timer.setStartTick();
				}
				positionprev=0;
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,0.0,velocity);
				avanza = true;			
			}else if (positionprev !=0) {
				positionprev=0;
				timer.setStartTick();
				avanza=false;
			}
		}
		else if(face_2d_position.x()>0.6){
			std::cout << "posPrev: " << positionprev << "timer: " << timer.time_m() << " avanza: " << avanza << " velocidad: " << velocity << std::endl;
			if ((positionprev==1) && (timer.time_m()>MAXTIME) ){
				//Situacion de giro
				angle_user += 1.5 * MPI / 180;
				positionprev=1;
				avanza=false;
			}
			else if ((positionprev !=1 && timer.time_m()<MAXTIME) || avanza){
				//Situación de avance
				if (positionprev != 1){ 
					velocity = media_velocity/timer.time_s();
					timer.setStartTick();
				}
				positionprev=1;
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,0.0,velocity);
				avanza = true;
			}
			else if (positionprev != 1){
				positionprev=1;
				timer.setStartTick();
				avanza=false;
			}
		}else{
			// Cabeza en el centro
			if (avanza)
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,0.0,velocity);
			if (positionprev != 0.5){
				positionprev=0.5;
				timer.setStartTick();
			}else if (positionprev == 0.5  && timer.time_m() > MAXTIME){
				avanza=false;
			}
		}

		// Agacharse
		if (face_2d_position.y()>0.7){
			if (!agachado){
				std::cout << " agachado: " << std::endl;
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,altura,0.0);
				agachado=true;
			}
		// Saltar
		}else if (face_2d_position.y()<0.4){
			if(!saltar){
				std::cout << " SALTA: " << std::endl;
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,-altura,velocity);
				/*if (face_2d_position.y()<0.2){
					position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,-altura,velocity);
				}else{
					position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,-altura,0.0);
				}*/
				saltar=true;
			}
		// En medio
		}else{
			if (agachado){
				std::cout << " NO agachado: " << std::endl;
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,-altura,0.0);
				agachado=false;
			}
		}

		osg::Vec3d head_position(position);
		osg::Vec3d rotation_axis(0.0,1.0,0.0);
		osg::Vec3d z_axis(0.0,0.0,1.0);
		//std::cout << angle_user << std::endl;
		world->setAttitude(osg::Quat(angle_user, init_pos));
		
		world->setPosition(osg::Quat(angle_user,init_pos)*position_user); //+init_pos);
		world->setPosition(world->getPosition()+init_pos);
		
		// compute projection matrices given sceen and head positions
		setProjectionMatrix(lower_left, lower_right, upper_left, head_position, near_plane, far_plane);
		
		if(saltar){
				position_user += osg::Quat(-angle_user,init_pos)*osg::Vec3d (0.0,altura,0.0);
				saltar=false;
		}

		// do openscenegraph stuff
		viewer.frame();

		// sleep for a while
		OpenThreads::Thread::microSleep(10000);

		//count++; // Numero de iteraciones del bucle

	}

	// close tracker
	tracker.close();

	// wait for thread to shut down
	OpenThreads::Thread::microSleep(1000000);

	return 0;

}

// Compute projection matrices (asymmetric frustum) from screen and eye coordinates
void setProjectionMatrix(osg::Vec3d pa, osg::Vec3d pb, osg::Vec3d pc, osg::Vec3d pe, double n, double f)
{

	osg::Vec3d va, vb, vc;
	osg::Vec3d vr, vu, vn;
	double l, r, b, t, d;

	// Compute an orthonormal basis for the screen.
	vr = pb - pa;
	vu = pc - pa;
	vr.normalize();
	vu.normalize();
	vn = vr^vu;
	vn.normalize();

	// Compute the screen corner vectors.
	va = pa - pe;
	vb = pb - pe;
	vc = pc - pe;

	// Find the distance from the eye to screen plane.
	d = -va*vn;
	
	// Find the extent of the perpendicular projection.
	l = (vr*va) * n / d;
	r = (vr*vb) * n / d;
	b = (vu*va) * n / d;
	t = (vu*vc) * n / d;
	
	// set matrices
	viewer.getCamera()->setProjectionMatrixAsFrustum(l, r, b, t, n, f);
	viewer.getCamera()->setViewMatrix(osg::Matrix::translate(-pe[0],-pe[1],-pe[2]));

}


void createWorld(osg::Group* parent)
{
   // add floor
	osg::Geode* floor_geode = new osg::Geode;
	createFloor(floor_geode);
	osg::PositionAttitudeTransform* floorXForm;
	floorXForm = new osg::PositionAttitudeTransform();
	floorXForm->setPosition(osg::Vec3d(0,0,0));
	floorXForm->setDataVariance( osg::Object::STATIC );
	floorXForm->addChild(floor_geode);
	parent->addChild(floorXForm);

	// gate positions
	std::vector<osg::Vec3d> gate_positions;
	gate_positions.push_back(osg::Vec3d(0.,0.,-4.));
	gate_positions.push_back(osg::Vec3d(-0.5,0.,-10.));
	gate_positions.push_back(osg::Vec3d(3.,0.,-7.));
	gate_positions.push_back(osg::Vec3d(1.,0.,-15.));
   std::vector<double> gate_heights;
   gate_heights.push_back(1.5);
   gate_heights.push_back(0.5);
   gate_heights.push_back(0.2);
   gate_heights.push_back(1.5);

   double radius = 0.05;
   double sep = 2.;
   
   for (unsigned int i=0; i<gate_positions.size(); i++)
   {
      osg::Geode* gate_geode = new osg::Geode;
      createGate(gate_geode, radius, sep, gate_heights[i]);

      osg::PositionAttitudeTransform* gateXForm;
      gateXForm = new osg::PositionAttitudeTransform();
      gateXForm->setPosition(gate_positions[i]);
      gateXForm->setDataVariance( osg::Object::STATIC );
      gateXForm->addChild(gate_geode);
      parent->addChild(gateXForm);
   }

}

// create one target
void createFloor(osg::Geode* geode)
{
	double side = 40.;
	osg::Vec3 top_left(-side/2,0.,-side/2);
	osg::Vec3 bottom_left(-side/2,0.,side/2);
	osg::Vec3 bottom_right(side/2,0.,side/2);
	osg::Vec3 top_right(side/2,0.,-side/2);
	
   // load image
	osg::Image* img = osgDB::readImageFile("../../data/floor_texture.jpg");
	if (!img)
	{
		std::cout << "Couldn't load texture." << std::endl;
		return;
	}
	
	// setup texture
	osg::Texture2D* texture = new osg::Texture2D();
	texture->setDataVariance(osg::Object::STATIC);
	texture->setImage(img);

   float nb_slices = 40.;

   float dx = (top_right[0]-top_left[0])/nb_slices;
   float dy = (bottom_left[2]-top_left[2])/nb_slices;

   for (float i=0; i<nb_slices; i++)
   {
      for (float j=0; j<nb_slices; j++)
      {
	      // create geometry
	      osg::Geometry* geom = new osg::Geometry;
	
	      osg::Vec3Array* vertices = new osg::Vec3Array(4);
	      (*vertices)[0] = osg::Vec3(top_left[0]+j*dx, 0., top_left[2]+i*dy);  // top left
	      (*vertices)[1] = osg::Vec3(top_left[0]+j*dx, 0., top_left[2]+(i+1)*dy); // bottom left
	      (*vertices)[2] = osg::Vec3(top_left[0]+(j+1)*dx, 0., top_left[2]+(i+1)*dy); // bottom right
	      (*vertices)[3] = osg::Vec3(top_left[0]+(j+1)*dx, 0., top_left[2]+i*dy); // top right
	      geom->setVertexArray(vertices);
	
	      osg::Vec2Array* texcoords = new osg::Vec2Array(4);
         float tiling = 1;
	      (*texcoords)[0].set(0.0f, 0.0f);
	      (*texcoords)[1].set(tiling, 0.0f);
	      (*texcoords)[2].set(tiling, tiling);
	      (*texcoords)[3].set(0.0f, tiling);
	      geom->setTexCoordArray(0,texcoords);
	
	      osg::Vec3Array* normals = new osg::Vec3Array(1);
	      (*normals)[0].set(0.,1.,0.);
	      geom->setNormalArray(normals);
	      geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	
	      osg::Vec4Array* colors = new osg::Vec4Array(1);
	      (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
	      geom->setColorArray(colors);
	      geom->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	      geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	      // setup state
	      osg::StateSet* state = geom->getOrCreateStateSet();
	      state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
	
	      geode->addDrawable(geom);
      }
   }

}


void createGate(osg::Geode* geode, double width, double sep, double height)
{
   // load image
	osg::Image* img = osgDB::readImageFile("../../data/grey_texture.jpg");
	if (!img)
	{
		std::cout << "Couldn't load texture." << std::endl;
		return;
	}
	
	// setup texture
	osg::Texture2D* texture = new osg::Texture2D();
	texture->setDataVariance(osg::Object::STATIC);
	texture->setImage(img);
  
   osg::Quat cyl_rot(osg::PI/2., osg::Vec3d(1,0,0));
   osg::Quat cyl_top_rot(osg::PI/2., osg::Vec3d(0,1,0));

   osg::Cylinder* cyl_left = new osg::Cylinder(osg::Vec3d(0,0,0),width, height);
   cyl_left->setRotation(cyl_rot);
   cyl_left->setCenter(osg::Vec3(-sep/2., height/2., 0.));
   osg::ShapeDrawable* cyl_left_drawable = new osg::ShapeDrawable(cyl_left);

   osg::Cylinder* cyl_right = new osg::Cylinder(osg::Vec3d(0,0,0),width, height);
   cyl_right->setRotation(cyl_rot);
   cyl_right->setCenter(osg::Vec3(sep/2., height/2., 0.));
   osg::ShapeDrawable* cyl_right_drawable = new osg::ShapeDrawable(cyl_right);

   osg::Cylinder* cyl_top = new osg::Cylinder(osg::Vec3d(0,0,0),width, sep);
   cyl_top->setRotation(cyl_top_rot);
   cyl_top->setCenter(osg::Vec3(0., height, 0.));
   osg::ShapeDrawable* cyl_top_drawable = new osg::ShapeDrawable(cyl_top);

   geode->addDrawable(cyl_left_drawable);
   geode->addDrawable(cyl_right_drawable);
   geode->addDrawable(cyl_top_drawable);

   osg::StateSet* state = geode->getOrCreateStateSet();
   state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

}