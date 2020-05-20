/*
	ball.cpp 
	Miguel Leitao, 2012, 2019
*/

#include <osgViewer/Viewer> 
#include <osg/Material>
#include "btosg/btosg.h"

#define _DEBUG_ (0)

int ResetFlag=0;
double frame_time = 0.;

// Create World
btosgWorld myWorld;

btosgSphere *myBall;

class BowlingPin : public btosgObject {
private:
    /* data */
    float height = 0.0;
    float rbody = 0.0;
public:
    BowlingPin(){
        mass = 1.55;
        height = 0.4; // Altura do pino, em metros
        rbody = 0.0655; // Raio maximo da barriga
        // Model
        loadObjectModel("pino.obj");
        model->setNodeMask(CastsShadowTraversalMask);
        
        // Colision Shape
        shape = new btCylinderShapeZ( btVector3(rbody, rbody, height/2.) );
        shape->setMargin( 0.0002 ) ;
        btVector3 inertia(0,0,0);
        shape->calculateLocalInertia(mass, inertia);
        
        // Rigid Body
        btDefaultMotionState* mState = new
        btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0.,0.,0.)));
        btRigidBody::btRigidBodyConstructionInfo cInfo(mass,mState,shape,inertia);
        cInfo.m_restitution = 0.75f;
        cInfo.m_friction = 0.3f;
        body = new btRigidBody(cInfo);
        if ( !body ) fprintf(stderr, "Error creating btBody for BowlingPin\n");
        else body->setDamping(0., 0.2);
    }
};
    
BowlingPin *myPin;

int main()
{
    btosgVec3 up(0., 0., 1.);
    btosgVec3 gravity = up*-9.8;
    myWorld.dynamic->setGravity(gravity);
    
    // Beach Ball
    myBall = new btosgSphere(0.1085);
    myBall->setMass(7);
    myBall->setTexture("beachball.png");
    myBall->setPosition(0, -6, 2);
    myWorld.addObject( myBall );
    
    myPin = new BowlingPin();
    myPin->setName("pino");
    myPin->setPosition(0., -0.4, 0.2);
    myWorld.addObject( myPin );
    
    // Material for planes
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);

    // Plane 1
    btosgPlane *myRamp;
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(-osg::PI/8.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,-5.,0.);
    myRamp->setName("Ramp1");
    myRamp->body->setFriction(100.);
    myRamp->model->getOrCreateStateSet()->
	setAttributeAndModes(matRamp, osg::StateAttribute::ON);  
    myWorld.addObject( myRamp );

    // Plane 2
    myRamp = new btosgPlane();
    myRamp->setRotation(osg::Quat(0.,osg::Vec3(1.,0.,0.)));
    myRamp->setPosition(0.,0.,0.);
    myRamp->setName("Ramp2");
    myRamp->body->setFriction(100.);
    myRamp->model->getOrCreateStateSet()->
	setAttributeAndModes(matRamp, osg::StateAttribute::ON);
    myWorld.addObject( myRamp );

    // Creating the viewer
    osgViewer::Viewer viewer ;

    // Setup camera
    osg::Matrix matrix;
    matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,1.), up );
    viewer.getCamera()->setViewMatrix(matrix);

    // Light
    osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
    ls->getLight()->setPosition(osg::Vec4(2.5,-10+30*up[1],-10+30.*up[2],1.)); 
    ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
    ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
    myWorld.scene->addChild(ls.get());

    viewer.setSceneData( myWorld.scene );

    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR ); 
        
    // Manipulator
    osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( manipulator );
    // Set the desired home coordinates for the manipulator
    osg::Vec3d eye(osg::Vec3(-15., 0., -5.)+up*20.);
    osg::Vec3d center(0., 0., 0.);
    // Make sure that OSG is not overriding our home position
    manipulator->setAutoComputeHomePosition(false);
    // Set the desired home position of the Trackball Manipulator
    manipulator->setHomePosition(eye, center, up);
    // Force the camera to move to the home position
    manipulator->home(0.0);
        
    // record the timer tick at the start of rendering.
    osg::Timer myTimer;
    double timenow = myTimer.time_s();
    double last_time = timenow;
    frame_time = 0.;

    while( !viewer.done() )
    {
	 	myWorld.stepSimulation(frame_time,10);
                
	  	viewer.frame();
	  	timenow = myTimer.time_s();
	  	frame_time = timenow - last_time;
	  	last_time = timenow;
        
		if (ResetFlag>0) {
                    myWorld.reset();
		    ResetFlag = 0;
		}
    }
}

