//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 1292 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "math.h"
#include <chrono>
//    cMultiMesh::getMesh();
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"

#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display and all the variables it will need for movement and collision detection
cVector3d cameraPos;
cVector3d viewVector;
cShapeSphere* camSphere;
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a virtual object
cMultiMesh* object;

// rotational velocity of the object
cVector3d rotVel;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// the system time last time a frame was updated
unsigned long lastMilliseconds;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;

// display level for collision tree
int collisionTreeDisplayLevel = 0;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())
#define PI 3.14159265358979
#define sensitivity 500
#define speed 0.0002
#define ZERO 0.0


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

cVector3d getRotationAngles(cVector3d direction);

// main haptics simulation loop
void updateHaptics(void);


//==============================================================================
/*
    DEMO:    object.cpp

    This demonstration loads a 3D mesh file by using the file loader
    functionality of the cMesh class. A finger-proxy algorithm is used to
    render the forces. Take a look at this example to understand the
    different functionalities offered by the tool force renderer.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
    Finally, a simple dynamics model is used to simulate the behavior
    of the object.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "DH2626 - Lab 2" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Texture   (ON/OFF)" << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[3] - Collision tree (ON/OFF)" << endl;
    cout << "[+] - Increase collision tree display depth" << endl;
    cout << "[-] - Decrease collision tree display depth" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    }

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);
    glewInit();
    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    // global vector object containing the position of the camera.
    cameraPos = cVector3d (0.0, 0.0, 0.0);
    double camSphereRadius = 0.02;
    viewVector = cVector3d(-1.0, -0.1, -0.1);
    world->addChild(camera);
    camSphere = new cShapeSphere(camSphereRadius);
    world->addChild(camSphere);
    camSphere->setLocalPos(cameraPos);
    camSphere->m_material->setRed();
    camSphere->setTransparencyLevel(0.0, true, true);

    // position and orient the camera
    camera->set( cameraPos,    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 100);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(1.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.4, 0.4, 0.4);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(1.0, 1.0, 1.0);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.002;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setGreen();
    tool->m_hapticPoint->m_sphereProxy->m_material->setTransparencyLevel(0);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.3);

    // start the haptic tool
    tool->start();

    //--------------------------------------------------------------------------
    // CREATE OBJECT
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a virtual mesh
    object = new cMultiMesh();

    // add object to world
    world->addChild(object);

    // load an object file
    bool fileload;
    fileload = object->loadFromFile("pipeCentred.obj");

    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly" << endl;
        close();
        return (-1);
    }

    // disable culling so that faces are rendered on both sides
    object->setUseCulling(false);

    cMaterial m;
    cMaterial m1;
    m.setBlueCadet();
    m1.setGreenOlive();
    object->setMaterial(m);

     // compute a boundary box
    object->computeBoundaryBox(true);

    // show/hide bounding box
    object->setShowBoundaryBox(false);

    // compute collision detection algorithm
    object->createAABBCollisionDetector(camSphereRadius);

    // define a default stiffness for the object
    object->setStiffness(0.5 * maxStiffness, true);

    // define some haptic friction properties
    object->setFriction(0.1, 0.2, true);

    // enable display list for faster graphic rendering
    object->setUseDisplayList(true);

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0, 1.0, 1.0),
                                     cColorf(1.0, 1.0, 1.0),
                                     cColorf(0.8, 0.8, 0.8),
                                     cColorf(0.8, 0.8, 0.8));


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option 1: show/hide texture
    if (key == '1')
    {
        bool useTexture = object->getUseTexture();
        object->setUseTexture(!useTexture);
    }

    // option 2: enable/disable wire mode
    if (key == '2')
    {
        bool useWireMode = object->getMesh(0)->getWireMode();
        object->getMesh(0)->setWireMode(!useWireMode, true);
    }

    // option 3: show/hide collision detection tree
    if (key == '3')
    {
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        bool show = object->getShowCollisionDetector();
        object->setShowCollisionDetector(!show, true);
    }

    // option -:
    if (key == '-')
    {
        collisionTreeDisplayLevel--;
        if (collisionTreeDisplayLevel < 0) { collisionTreeDisplayLevel = 0; }
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        object->setShowCollisionDetector(true, true);
    }

    // option +:
    if (key == '+')
    {
        collisionTreeDisplayLevel++;
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        object->setShowCollisionDetector(true, true);
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

/**
 * @brief Calculates and returns appropriate values for the angles in the
 * rotation matrix.
 * @param dirVector The direction vector (should always be normalized).
 * @return The calculated angles.
 * @author Fredrik Johansson
 */
cVector3d getRotationAngles(cVector3d dirVector) {

    double xyAngle = 0, yzAngle = 0, zxAngle = 0;
    double vec;
    cVector3d* angles = new cVector3d(0,0,0);
    if(dirVector.y() < 0.95 || dirVector.y() > -0.95) {
    if(dirVector.x() < 0) {
        if(dirVector.y() != 0) {
            if(dirVector.y() < 0) {
            // Find x,y angle
                double x1 = -1, y1 = 0, x2 = dirVector.x(), y2 = dirVector.y();
                double sq = sqrt(x2 * x2 + y2 * y2);
                x2 = x2 / sq;
                y2 = y2 / sq;
                xyAngle = acos(x1*x2 + y1*y2);
            } else {
                double x1 = -1, y1 = 0, x2 = dirVector.x(), y2 = dirVector.y();
                double sq = sqrt(x2 * x2 + y2 * y2);
                x2 = x2 / sq;
                y2 = y2 / sq;
                xyAngle = -acos(x1*x2 + y1*y2);
            }
        }
    }

    if(camera->getLookVector().x() >= ZERO) {
        if(dirVector.x() < 0) {
            if(dirVector.z() != 0.0d) {
                if(dirVector.z() > 0) {
                    // Find z,x angle
                    double z1 = 0, z2 = dirVector.z(), x1 = -1, x2 = dirVector.x();
                    double sq = sqrt(z2*z2 + x2*x2);
                    z2 = z2 / sq;
                    x2 = x2 / sq;
                    zxAngle = acos(z1*z2 + x1*x2);
                } else {
                    // Find z,x angle
                    double z1 = 0, z2 = dirVector.z(), x1 = -1, x2 = dirVector.x();
                    double sq = sqrt(z2*z2 + x2*x2);
                    z2 = z2 / sq;
                    x2 = x2 / sq;
                    zxAngle = -acos(z1*z2 + x1*x2);
                }
            }
        } else {
            return *angles;
        }
    } else if(camera->getLookVector().x() < ZERO) {
        if(dirVector.x() < 0) {
            if(dirVector.z() != 0.0d) {
                if(dirVector.z() > 0) {
                    // Find z,x angle
                    double z1 = 0, z2 = dirVector.z(), x1 = -1, x2 = dirVector.x();
                    double sq = sqrt(z2*z2 + x2*x2);
                    z2 = z2 / sq;
                    x2 = x2 / sq;
                    zxAngle = -acos(z1*z2 + x1*x2);
                } else {
                    // Find z,x angle
                    double z1 = 0, z2 = dirVector.z(), x1 = -1, x2 = dirVector.x();
                    double sq = sqrt(z2*z2 + x2*x2);
                    z2 = z2 / sq;
                    x2 = x2 / sq;
                    zxAngle = acos(z1*z2 + x1*x2);
                }
            }
        } else {
            return *angles;
        }
    }

    angles->set(xyAngle/sensitivity, yzAngle/sensitivity, zxAngle/sensitivity);

    return *angles;
}
}

//------------------------------------------------------------------------------

/**
 * @brief Returns rotation matrix given three precalculated angles.
 * @param angles The precalculated angles.
 * @return The rotation matrix.
 * @author Fredrik Johansson
 */
cMatrix3d getRotationMatrix(cVector3d dirVector) {

    cVector3d angles = getRotationAngles(dirVector);

    cMatrix3d* rX = new cMatrix3d();
    cMatrix3d* rY = new cMatrix3d();
    cMatrix3d* rZ = new cMatrix3d();

    double yx = angles.x();
    double yz = angles.y();
    double zx = angles.z();

    rX->set(1, 0, 0, 0, cos(yz), -sin(yz), 0, sin(yz), cos(yz));
    rY->set(cos(zx), 0, sin(zx), 0, 1, 0, -sin(zx), 0, cos(zx));
    rZ->set(cos(yx), -sin(yx), 0, sin(yx), cos(yx), 0, 0, 0, 1);

    rZ->mul(*rY);
    rZ->mul(*rX);

    return *rZ;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setString ("haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;

    // change the camera view direction accordingly.
    camera->set(cameraPos, viewVector, cVector3d(0,0,1));
}

//------------------------------------------------------------------------------

enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics(void)
{


    // simulation clock
    cPrecisionClock simClock;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    bool isApplyingGravity = false;
    bool previousUserSwitch = false;

    // main haptic simulation loop
    while(simulationRunning)
    {

        /////////////////////////////////////////////////////////////////////////
        // MOVEMENT HANDLING
        /////////////////////////////////////////////////////////////////////////

        unsigned long milliseconds_since_epoch = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

        if(lastMilliseconds != milliseconds_since_epoch) {
            cVector3d currentPos = tool->getDeviceLocalPos();

            cVector3d position;

            hapticDevice->getPosition(position);

            // the direction equals to a vector where each dimension is set to the coordinates of the device location.
            cVector3d direction = position;

            // normalize the direction vector.
            direction.normalize();

            // get the rotation matrix for the (-1,0,0) and direction vectors divided by a constant set in the pre-processor.
            cMatrix3d rotationMatrix = getRotationMatrix(direction);

            // rotate the view using the rotation matrix.
            cVector3d newDirection = rotationMatrix * viewVector;

            // overwrite the old direction with the new one.
            viewVector = newDirection;

            cVector3d newPosition = cameraPos;

            // TODO we have to read the location of the proxy here. Maybe?
            // if button is pressed, do not change position of camera, if not pressed update position depending on how much we are pressing the device ahead.
            if (tool->getUserSwitch(0) == 1)
            {
                // Do nothing.
            } else {
                newPosition = cameraPos + camera->getLookVector()*currentPos.x()*speed;
                //tool->setLocalPos(newPosition);
            }

            cameraPos = newPosition;
            lastMilliseconds = milliseconds_since_epoch;
        }

        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // update frequency counter
        frequencyCounter.signal(1);



        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        hapticDevice->setForce(cVector3d(0, 0, 0));

        tool->computeInteractionForces();

        // send forces to haptic device
        if(!isApplyingGravity)
            tool->applyForces();
        isApplyingGravity = false;

        // retrieve and update the force that is applied on each object

        // stop the simulation clock
        simClock.stop();

        // read the time increment in seconds
        double timeInterval = simClock.getCurrentTimeSeconds();

        // restart the simulation clock
        simClock.reset();
        simClock.start();

        // temp variable to compute rotational acceleration
        cVector3d rotAcc(0,0,0);

        // check if tool is touching an object
        cGenericObject* objectContact = nullptr;
        if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
        {
            // get contact event
            cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

            // get object from contact event
            objectContact = collisionEvent->m_object;
        }

        if (objectContact != NULL)
        {
            cout << "collision";
        }



        /**
        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        if(!isApplyingGravity)
            tool->applyForces();
        isApplyingGravity = false;

        // retrieve and update the force that is applied on each object

        // stop the simulation clock
        simClock.stop();

        // read the time increment in seconds
        double timeInterval = simClock.getCurrentTimeSeconds();

        // restart the simulation clock
        simClock.reset();
        simClock.start();

        // temp variable to compute rotational acceleration
        cVector3d rotAcc(0,0,0);

        // check if tool is touching an object
        cGenericObject* objectContact = nullptr;
        if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
        {
            // get contact event
            cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

            // get object from contact event
            objectContact = collisionEvent->m_object;
        }

        if (objectContact != NULL)
        {
            // retrieve the root of the object mesh
            cGenericObject* obj = objectContact;

            // get position of cursor in global coordinates
            cVector3d toolPos = tool->getDeviceGlobalPos();

            // get position of object in global coordinates
            cVector3d objectPos = obj->getGlobalPos();

            // compute a vector from the center of mass of the object (point of rotation) to the tool
            cVector3d vObjectCMToTool = cSub(toolPos, objectPos);

            // compute acceleration based on the interaction forces
            // between the tool and the object
            if (vObjectCMToTool.length() > 0.0)
            {
                // get the last force applied to the cursor in global coordinates
                // we negate the result to obtain the opposite force that is applied on the
                // object
                cVector3d toolForce = cNegate(tool->m_lastComputedGlobalForce);

                // compute effective force to take into account the fact the object
                // can only rotate around a its center mass and not translate
                cVector3d effectiveForce = toolForce - cProject(toolForce, vObjectCMToTool);

                // compute the resulting torque
                cVector3d torque = cMul(vObjectCMToTool.length(), cCross( cNormalize(vObjectCMToTool), effectiveForce));

                // update rotational acceleration
                const double OBJECT_INERTIA = 1;
                rotAcc = (1.0 / OBJECT_INERTIA) * torque;
            }
        }

        // update rotational velocity
        rotVel.add(timeInterval * rotAcc);

        // set a threshold on the rotational velocity term
        const double ROT_VEL_MAX = 10.0;
        double velMag = rotVel.length();
        if (velMag > ROT_VEL_MAX)
        {
            rotVel.mul(ROT_VEL_MAX / velMag);
        }

        // add some damping too
        const double DAMPING_GAIN = 0.1;
        rotVel.mul(1.0 - DAMPING_GAIN * timeInterval);

        // if user switch is pressed, set velocity to zero
        if (tool->getUserSwitch(0) == 1)
        {
            rotVel.zero();
           // cVector3d force= cVector3d(0,0,4);
           // tool->getHapticDevice()->setForce(force);
        }
        cVector3d offset;
        if (tool->getUserSwitch(0) == 1 && !previousUserSwitch && objectContact != NULL)
        {

            cVector3d toolPos = tool->getDeviceGlobalPos();
            cVector3d objectPos = objectContact->getGlobalPos();
            offset = cSub(toolPos, objectPos);

        }

        if (tool->getUserSwitch(0) == 1 && objectContact != NULL)
        {
            //object touched & user switch pressed
           cVector3d force= cVector3d(0,0,-2);
           tool->getHapticDevice()->setForce(force);
           isApplyingGravity = true;
           objectContact->getParent()->setLocalPos(tool->getDeviceGlobalPos() - offset);
        }

        // compute the next rotation configuration of the object
        if (rotVel.length() > chai3d::C_SMALL)
        {
            object->rotateAboutGlobalAxisRad(cNormalize(rotVel), timeInterval * rotVel.length());
        }

        previousUserSwitch =tool->getUserSwitch(0);
        */


    }

    // exit haptics thread
    simulationFinished = true;
}

















