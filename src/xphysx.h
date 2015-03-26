#include "PxPhysicsAPI.h"
#include "PxDefaultAllocator.h"
#include "PxDefaultErrorCallback.h"
#include "PxTolerancesScale.h"
#include "PxDefaultSimulationFilterShader.h"
#include "PxExtensionsAPI.h"
#include "PxDefaultCpuDispatcher.h"
#include "PxShapeExt.h"
#include "PxTask.h"
#include "ofMain.h"

#pragma comment(lib, "PhysX3_x64.lib")
#pragma comment(lib, "PhysX3Common_x64.lib")
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "PxTask.lib")

using namespace physx;
//using namespace std;

PxPhysics* gPhysicsSDK = NULL;
PxDefaultErrorCallback gDefaultErrorCallback;
PxDefaultAllocator gDefaultAllocatorCallback;
PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;

typedef GLfloat point3[3];
point3 planeVertices[4] = { {-10,0,10},{-10,0,-10},{10,0,-10},{10,0,10} };

int startTime = 0, totalFrames = 0;
float fps = 0;
const float globalGravity = -20.f;
const int objnum = 150;

PxScene* gScene = NULL;
PxReal myTimestep = 1.0f/60.0f;
vector <PxRigidActor*> boxes;
vector <PxRigidActor*> spheres;

GLfloat lightAmbientColour[] = { 0.4f, 0.4f, 0.4f, 1.0f };
GLfloat lightDiffuseColour[] = { 0.8f, 0.8f, 0.8f, 1.0f };
GLfloat lightSpecularColour[] = { 0.8f, 0.8f, 0.8f, 1.0f };

void DrawPlane(){
    glBegin(GL_POLYGON);
    glNormal3f(0,1,0);
    glVertex3fv(planeVertices[0]);
    glVertex3fv(planeVertices[1]);
    glVertex3fv(planeVertices[2]);
    glVertex3fv(planeVertices[3]);
    glEnd();
}

void stepPX() {
    gScene->simulate(myTimestep);
    while(!gScene->fetchResults() ) {
        // do something useful
    }
}

void initPX() {
    PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    if(!foundation)
        ofLog(OF_LOG_ERROR)<<"PxCreateFoundation failed!"<<endl;

    gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale() );
    if(gPhysicsSDK == NULL){
        ofLog(OF_LOG_ERROR)<<"Error creating PhysX3 device."<<endl;
        ofLog(OF_LOG_ERROR)<<"Exiting..."<<endl;
        ofExit(1);
    }
    PxInitExtensions(*gPhysicsSDK);
    if(!PxInitExtensions(*gPhysicsSDK))
        ofLog(OF_LOG_ERROR)<<"PxInitExtensions failed!"<<endl;
    PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, globalGravity, 0.0f);
    if(!sceneDesc.cpuDispatcher) {
        PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
        sceneDesc.cpuDispatcher = mCpuDispatcher;
    }
    if(!sceneDesc.filterShader)
        sceneDesc.filterShader  = gDefaultFilterShader;
    gScene = gPhysicsSDK->createScene(sceneDesc);
    if(!gScene)
        ofLog(OF_LOG_ERROR)<<"createScene failed!"<<endl;
    PxMaterial* planeMaterial = gPhysicsSDK->createMaterial(0.1f, 0.1f, 1.0f);
    PxMaterial* cubeMaterial = gPhysicsSDK->createMaterial(1.f, 0.f, 0.f);
    PxMaterial* sphereMaterial = gPhysicsSDK->createMaterial(1.f, 0.5f, 0.f);

    // *** Create Ground-Plane *** //
    PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
    PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
    PxShape* shape = plane->createShape(PxPlaneGeometry(), *planeMaterial);
    gScene->addActor(*plane);

    // *** Create Wall-Planes *** //
    PxTransform leftPose = PxTransform(PxVec3(0.0f, 0.0f, 10.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));
    PxRigidStatic* leftPlane = gPhysicsSDK->createRigidStatic(leftPose);
    PxShape* leftShape = leftPlane->createShape(PxPlaneGeometry(), *planeMaterial);
    gScene->addActor(*leftPlane);

    PxTransform rightPose = PxTransform(PxVec3(-10.0f, 0.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f)));
    PxRigidStatic* rightPlane = gPhysicsSDK->createRigidStatic(rightPose);
    PxShape* rightShape = rightPlane->createShape(PxPlaneGeometry(), *planeMaterial);
    gScene->addActor(*rightPlane);

    PxTransform leftPose2 = PxTransform(PxVec3(0.0f, 0.0f, -10.0f), PxQuat(PxHalfPi, PxVec3(0.0f, -1.0f, 0.0f)));
    PxRigidStatic* leftPlane2 = gPhysicsSDK->createRigidStatic(leftPose2);
    PxShape* leftShape2 = leftPlane2->createShape(PxPlaneGeometry(), *planeMaterial);
    gScene->addActor(*leftPlane2);

    PxTransform rightPose2 = PxTransform(PxVec3(10.0f, 0.0f, 0.0f), PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f)));
    PxRigidStatic* rightPlane2 = gPhysicsSDK->createRigidStatic(rightPose2);
    PxShape* rightShape2 = rightPlane2->createShape(PxPlaneGeometry(), *planeMaterial);
    gScene->addActor(*rightPlane2);

    // Init Cube Actor
    PxReal cubeDensity = 2.0f;
    PxTransform cubeTransform(PxVec3(0.0f, 4.0, 0.0f));
    PxVec3 cubeDims(0.5,0.5,0.5);
    PxBoxGeometry cubeGeometry(cubeDims);
    for (int i=0; i<objnum; i++) {
        cubeTransform.p  = PxVec3(0.0f,4.0f+5*i,0.0f);
        PxRigidDynamic *cubeActor = PxCreateDynamic(*gPhysicsSDK, cubeTransform, cubeGeometry, *cubeMaterial, cubeDensity);
        if (!cubeActor)
            ofLog(OF_LOG_ERROR)<<"create actor failed!"<<endl;
        cubeActor->setAngularDamping(0.2);
        cubeActor->setLinearDamping(0.01);
        cubeActor->setMass(1+(i/8));
        gScene->addActor(*cubeActor);
        boxes.push_back(cubeActor);
    }
    // Init Sphere Actor
    PxReal sphereDensity = 7.0f;
    PxTransform sphereTransform(PxVec3(0.0f, 4.0f, 0.0f));
    PxSphereGeometry sphereGeometry(0.7f);
    for (int i=0; i<objnum; i++) {
        sphereTransform.p  = PxVec3(0.0f,4.0f+4*i,0.0f);
        PxRigidDynamic *sphereActor = PxCreateDynamic(*gPhysicsSDK, sphereTransform, sphereGeometry, *sphereMaterial, sphereDensity);
        if (!sphereActor)
            ofLog(OF_LOG_ERROR)<<"create actor failed!"<<endl;
            sphereActor->setAngularDamping(0.2);
        sphereActor->setLinearDamping(0.01);
        sphereActor->setMass(1+(i/4));
        gScene->addActor(*sphereActor);
        spheres.push_back(sphereActor);
    }
}

void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
    mat[0] = m.column0[0];
    mat[1] = m.column0[1];
    mat[2] = m.column0[2];
    mat[3] = 0;

    mat[4] = m.column1[0];
    mat[5] = m.column1[1];
    mat[6] = m.column1[2];
    mat[7] = 0;

    mat[8] = m.column2[0];
    mat[9] = m.column2[1];
    mat[10] = m.column2[2];
    mat[11] = 0;

    mat[12] = t[0];
    mat[13] = t[1];
    mat[14] = t[2];
    mat[15] = 1;
}

void DrawBox(PxShape* pShape,PxRigidActor * actor) {
    physx::PxTransform pT =  PxShapeExt::getGlobalPose(*pShape,*actor);
    PxBoxGeometry bg;
    pShape->getBoxGeometry(bg);
    PxMat33 m = PxMat33(pT.q);
    float mat[16];
    getColumnMajor(m,pT.p, mat);
    ofPushMatrix();
    ofMultMatrix(mat);
    ofDrawBox(bg.halfExtents.x*2);
    ofPopMatrix();
}

void DrawSphere(PxShape* pShape, PxRigidActor * actor){
    physx::PxTransform pT = PxShapeExt::getGlobalPose(*pShape,*actor);
    PxSphereGeometry bg;
    pShape->getSphereGeometry(bg);
    PxMat33 m = PxMat33(pT.q );
    float mat[16];
    getColumnMajor(m,pT.p, mat);
    ofPushMatrix();
    ofMultMatrix(mat);
    ofDrawSphere(bg.radius);
    ofPopMatrix();
}

void DrawShape(PxShape* shape,PxRigidActor *actor){
    PxGeometryType::Enum type = shape->getGeometryType();
    switch(type) {
        case PxGeometryType::eBOX:
            DrawBox(shape,actor);
        break;
        case PxGeometryType::eSPHERE:
            DrawSphere(shape,actor);
        break;
    }
}

void DrawActor(PxRigidActor* actor) {
    PxU32 nShapes = actor->getNbShapes();
    PxShape** shapes = new PxShape*[nShapes];
    actor->getShapes(shapes, nShapes);
    while (nShapes--)
    DrawShape(shapes[nShapes],actor);
    delete [] shapes;
}

void RenderActors() {
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    // Stage Lighting
    GLfloat lightPosition[4] = {20*cos(0.0),20*sin(0.0),0.0,1.0};
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lightAmbientColour);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,lightDiffuseColour);
    glLightfv(GL_LIGHT0,GL_SPECULAR,lightSpecularColour);
    glLightfv(GL_LIGHT0,GL_POSITION, lightPosition);

    GLfloat ambient_cube[]={0.0f,0.0f,0.30f,0.40f};
    GLfloat ambient_sphere[]={0.20f,0.20f,0.0f,0.25f};

    GLfloat diffuse_cube[]={1.0f,1.0f,1.0f,1.0f};
    GLfloat diffuse_sphere[]={1.0f,1.0f,1.0f,1.0f};

    GLfloat mat_diffuse_cube[]={0.85f, 0.0f, 0.0f, 1.0f};
    GLfloat mat_diffuse_sphere[]={0.85f, 0.85f, 0.0f, 1.0f};

    for (int i=0;i<objnum;i++) {
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diffuse_cube);
        DrawActor(boxes[i]);
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diffuse_sphere);
        DrawActor(spheres[i]);
    }
    glDisable(GL_LIGHTING);
}

void shutdownPX(){
    gScene->release();
    gPhysicsSDK->release();
}

void renderPX() {
    GLdouble viewer[3]= {20*sin(0.0),20,20*cos(0.0)};
    totalFrames++;
    int current = ofGetElapsedTimeMillis();
    if((current-startTime)>1000) {
        float elapsedTime = float(current-startTime);
        fps = ((totalFrames * 1000.0f)/ elapsedTime);
        startTime = current;
        totalFrames = 0;
    }
    if (gScene)
        stepPX();

    ofTranslate(0,0,200);
    ofRotate(0,1,0,0);
    ofRotate(50,0,1,0);

    ofDrawAxis(5.f);
    ofDrawGrid(10.0f,8.0f,false,false,true,false);

    ofPushMatrix();
    ofTranslate(0,10,10);
    ofRotate(90,1,0,0);
    DrawPlane();
    ofPopMatrix();

    ofPushMatrix();
    ofTranslate(0,10,-10);
    ofRotate(-90,1,0,0);
    DrawPlane();
    ofPopMatrix();

    ofPushMatrix();
    ofTranslate(0,10,0);
    ofRotate(90,0,1,0);
    ofRotate(-90,1,0,0);
    ofTranslate(0,10,0);
    DrawPlane();
    ofPopMatrix();

    RenderActors();
}
/* end */
