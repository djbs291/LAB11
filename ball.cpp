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

    float rhead = 0.04;
    float rbase = 0.048;
    float rneck = 0.035;
    float zbody = -0.05;

public:
    BowlingPin(){
        mass = 1.55;
        height = 0.4; // Altura do pino, em metros
        rbody = 0.0655; // Raio maximo da barriga
        // Model

        loadObjectModel("pino.obj");
        model->setNodeMask(CastsShadowTraversalMask);

        btCompoundShape* cShape = new btCompoundShape();
        if ( !cShape ) fprintf(stderr,"Error creating btCompoundShape\n");
        // Neck
        cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)),
            new btCylinderShapeZ(btVector3(rneck, rneck, height/2)) );
        // Body
        cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,zbody)),
            new btSphereShape(rbody) );
        // Base
        cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,-height/4.)),
            new btCylinderShapeZ(btVector3(rbase, rbase, height/4.)) );
        // Head
        cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,height/2-rhead)), 
            new btSphereShape(rhead) );
        shape = cShape;

        // Colision Shape
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

// class to handle events
class EventHandler : public osgGA::GUIEventHandler
{
    public:
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer) return false;
        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
                switch ( ea.getKey() ) {
                    case 'S':
                        std::cout << "tecla S" << std::endl;
                        return false;
                    case 'r':
                        ResetFlag = 1;
                        break;
                    case osgGA::GUIEventAdapter::KEY_Down:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(0.,-0.5,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Up:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(0.,1.,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Left:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(-0.5,0,0.));
                        return false;
                    case osgGA::GUIEventAdapter::KEY_Right:
                        myBall->body->activate(true);
                        myBall->body->applyCentralImpulse(btVector3(0.5,0,0.));
                        return false;
                }
            case(osgGA::GUIEventAdapter::MOVE):
                std::cout << "mouse move" << ea.getX()<< " " << ea.getY()<< std::endl;
                return false;
            default:
                return false;
        }
    }
};

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

    BowlingPin *myPin[10];
    int x, y, p = 0;
    float space = 12*0.0254; // 12 inches
    for(y=0;y<4;y++){
        for(x=0;x<=y;x++){
            myPin[p] = new BowlingPin();
            myPin[p] -> setName("pin");
            myPin[p] -> setPosition(0.+(x-y/2.)*space, -1.4+(y)*space, 0.20);
            myWorld.addObject(myPin[p]);
            p+=1;
        }
    }

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

    // Add Light Source
    osg::LightSource *ls = new osg::LightSource;
    ls->getLight()->setPosition(osg::Vec4(0.5, -0.7, 1., 0.));
    ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1.));
    myWorld.scene->addChild(ls);

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

    viewer.addEventHandler(new EventHandler());
        
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

