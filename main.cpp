/*
 *  CSCI 441, Computer Graphics, Fall 2017
 *
 *  Project: lab02
 *  File: main.cpp
 *
 *	Author: Dr. Jeffrey Paone - Fall 2015
 *	Modified: Dr. Jeffrey Paone - Fall 2017 for GLFW
 *
 *  Description:
 *      Contains the base code for a basic flight simulator.
 *
 */

// include the OpenGL library header
#ifdef __APPLE__					// if compiling on Mac OS
	#include <OpenGL/gl.h>
#else										// if compiling on Linux or Windows OS
	#include <GL/gl.h>
#endif

#include <GLFW/glfw3.h>	// include GLFW framework header

#include <CSCI441/objects.hpp> // for our 3D objects

// include GLM libraries and matrix functions
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <math.h>				// for cos(), sin() functionality
#include <stdio.h>			// for printf functionality
#include <stdlib.h>			// for exit functionality
#include <time.h>			  // for time() functionality

#include <fstream>			// for file I/O
#include <vector>				// for vector

#include <iostream>
#include <string>
using namespace std;

//*************************************************************************************
//
// Global Parameters

#ifndef M_PI
#define M_PI 3.14159
#endif

// global variables to keep track of window width and height.
// set to initial values for convenience, but we need variables
// for later on in case the window gets resized.
int windowWidth = 640, windowHeight = 480;

int leftMouseButton;    	 						// status of the mouse button
glm::vec2 mousePos;			              		  	// last known X and Y of the mouse

glm::vec3 camPos;            						// camera position in cartesian coordinates
float cameraTheta, cameraPhi;               		// camera DIRECTION in spherical coordinates
glm::vec3 camDir; 			                    	// camera DIRECTION in cartesian coordinates

glm::vec3 lookAt;                                   // What the camera is looking at

GLuint environmentDL;                       		// display list for the 'city'

float rotate = 0.0f;

float turnHead = 0.0f;

float scale = 8.0f;

bool holdingControl = false;


vector<glm::vec3> controlPoints;

vector<glm::vec3> controlSurf; //bezier surface


float trackPointVal = 0.0f;
int numSegments = 0;

int resolution = 150;
vector<glm::vec3> points;

int res_u = 80;
int res_v = 150;
vector<glm::vec3> surfPoints;



float ridePos = 0;

bool hideControlCage = false;
bool hideBezier = false;

bool c1 = false;

//*************************************************************************************
//
// Helper Functions

// getRand() ///////////////////////////////////////////////////////////////////
//
//  Simple helper function to return a random number between 0.0f and 1.0f.
//
////////////////////////////////////////////////////////////////////////////////
float getRand() { return rand() / (float)RAND_MAX; }


// loadControlPoints() /////////////////////////////////////////////////////////
//
//  Load our control points from file and store them in
//	the global variable controlPoints
//
////////////////////////////////////////////////////////////////////////////////
bool loadControlPoints( string filename ) {
 
    int num;
    string line;
    ifstream myfile;
    myfile.open( filename);
    if(myfile.is_open()){
        //cout << "test";
        getline(myfile, line);
        num = stoi(line);

        for(int i = 0; i < num; i++) {
            getline(myfile, line);
			//set xyz coords
            float x = stof(line.substr(0, line.find(',')));
            float y = stof(line.substr(line.find(',') + 1, line.find_last_of(',')));
            float z = stof(line.substr(line.find_last_of(',') + 1));
            controlPoints.push_back(glm::vec3(x, y, z));
           
        }
    }else{
        
        return false;
    }

    myfile.close();
    return true;
}


// loadSurfPoints() /////////////////////////////////////////////////////////
//
//	Load surface file and generate points to surfPoints vector
//
////////////////////////////////////////////////////////////////////////////////
bool loadSurfPoints( string filename ) {
 
    int num;
    string line;
    ifstream myfile;
    myfile.open( filename);
    if(myfile.is_open()){
        //cout << "test";
        getline(myfile, line);
        num = stoi(line);

        for(int i = 0; i < num; i++) {
            getline(myfile, line);
			//set xyz coords
            float x = stof(line.substr(0, line.find(',')));
            float y = stof(line.substr(line.find(',') + 1, line.find_last_of(',')));
            float z = stof(line.substr(line.find_last_of(',') + 1));
            controlSurf.push_back(glm::vec3(x, y, z));
           
        }
    }else{
        
        return false;
    }

    myfile.close();
    return true;
}









// recomputeOrientation() //////////////////////////////////////////////////////
//
// This function updates the camera's direction in cartesian coordinates based
//  on its position in spherical coordinates. Should be called every time
//  cameraTheta or cameraPhi is updated.
//
////////////////////////////////////////////////////////////////////////////////
void recomputeOrientation() {
	// TODO #5: Convert spherical coordinates into a cartesian vector
   camDir = glm::vec3(sin(cameraTheta)*sin(cameraPhi), -cos(cameraPhi), -cos(cameraTheta)*sin(cameraPhi));

	// and NORMALIZE this directional vector!!!
	float normalize = sqrt(camDir.x*camDir.x + camDir.y*camDir.y + camDir.z*camDir.z);
    camDir = glm::vec3(camDir.x/normalize, camDir.y/normalize, camDir.z/normalize);
}

bool ring = false;


// evaluateBezierCurve() ////////////////////////////////////////////////////////
//
// Computes a location along a Bezier Curve.
//
////////////////////////////////////////////////////////////////////////////////
glm::vec3 evaluateBezierCurve( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, float t ) {
    glm::vec3 point(0,0,0);
    glm::vec3 a = (-p0 + 3.0f*p1 -3.0f*p2 + p3);
    glm::vec3 b = (3.0f*p0 - 6.0f*p1 + 3.0f*p2);
    glm::vec3 c = (-3.0f*p0 + 3.0f*p1);
    glm::vec3 d = p0;

    point = a*(t*t*t) + b*(t*t) + c*t + d;
    

    return point;
}



// evaluateBezierSurf() ////////////////////////////////////////////////////////
//
// Computes a location along a Bezier Curve.
//
////////////////////////////////////////////////////////////////////////////////
glm::vec3 evaluateBezierSurf(vector <glm::vec3> Pts , float u, float v) { //takes array of 15
	glm::vec3 pt(0, 0, 0);

	glm::vec3 p_1 = evaluateBezierCurve(Pts[0], Pts[1], Pts[2], Pts[3], u);
	glm::vec3 p_2 = evaluateBezierCurve(Pts[4], Pts[5], Pts[6], Pts[7], u);
	glm::vec3 p_3 = evaluateBezierCurve(Pts[8], Pts[9], Pts[10], Pts[11], u);
	glm::vec3 p_4= evaluateBezierCurve(Pts[12], Pts[13], Pts[14], Pts[15], u);

	pt = p_1 * powf((1 - v), 3);
	pt += 3 * v * p_2 * powf((1 - v), 3);
	pt += 3 * powf(v, 2) * p_3 * powf((1 - v), 3);
	pt += powf(v,3)*p_4 *(1 - v);
	

	return pt;
}







// renderBezierCurve() //////////////////////////////////////////////////////////
//
// Responsible for drawing a Bezier Curve as defined by four control points.
//  Breaks the curve into n segments as specified by the resolution.
//
////////////////////////////////////////////////////////////////////////////////


void renderBezierCurve( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, int resolution ) {
    
    glDisable( GL_LIGHTING );
    glColor3f(0,0,1);
    glLineWidth(3.0f);
    glBegin(GL_LINE_STRIP);
    float temp = 1.0/float(resolution);
    for(int i = 0; i <= resolution; i++){

        glm::vec3 newPoint = evaluateBezierCurve(p0,p1,p2,p3,temp*i);



        if(!hideBezier) {
            glVertex3f(newPoint.x, newPoint.y, newPoint.z);
        }


        points.push_back(glm::vec3(newPoint.x, newPoint.y, newPoint.z));
    }
    glEnd();
    glEnable(GL_LIGHTING);
}



// renderBezierSurf() //////////////////////////////////////////////////////////
//
// Functions like renderBezierCurve()
//
//////////////////////////////////////////////////////////////////////////////////


void renderBezierSurf(vector <glm::vec3> pts, int res_u, int res_v) {

	glDisable(GL_LIGHTING);
	glColor3f(.6, 0, .5); //dark, spacey purple
	glLineWidth(3.0f);
	glBegin(GL_LINE_STRIP);
	
	// 0 <= temp <= 1
	float temp_u = 1.0 / float(res_u); 
	float temp_v = 1.0 / float(res_v);


	for (int i = 0; i <= resolution; i++) {

		glm::vec3 newPoint = evaluateBezierSurf(pts, temp_u * i ,temp_v * i);



		if (!hideBezier) {
			glVertex3f(newPoint.x, newPoint.y, newPoint.z);
		}


		surfPoints.push_back(glm::vec3(newPoint.x, newPoint.y, newPoint.z));
	}
	glEnd();
	glEnable(GL_LIGHTING);
}
















//*************************************************************************************
//
// Event Callbacks

//
//	void error_callback( int error, const char* description )
//
//		We will register this function as GLFW's error callback.
//	When an error within GLFW occurs, GLFW will tell us by calling
//	this function.  We can then print this info to the terminal to
//	alert the user.
//
static void error_callback( int error, const char* description ) {
	fprintf( stderr, "[ERROR]: %s\n", description );
}

float pos = 1;

void checkPos(){
    if(lookAt.x >= 50){
        lookAt.x = 50;
    }else if(lookAt.x <= -50){
        lookAt.x = -50;
    }
    if(lookAt.z >= 50) {
        lookAt.z = 50;
    }else if(lookAt.z <= -50){
        lookAt.z = -50;
    }
    if(lookAt.y >= 50) {
        lookAt.y = 50;
    }else if(lookAt.y <= 0){
        lookAt.y = 0;
    }
}


static void keyboard_callback( GLFWwindow *window, int key, int scancode, int action, int mods ) {
	if( action == GLFW_PRESS ) {
		switch( key ) {
			case GLFW_KEY_ESCAPE:
			case GLFW_KEY_Q:
				exit(EXIT_SUCCESS);
            case GLFW_KEY_0:
                c1 = false;
                break;
            case GLFW_KEY_1:
                c1 = true;
                break;
            case GLFW_KEY_2:
                hideControlCage = !hideControlCage;
                break;
            case GLFW_KEY_3:
                hideBezier = !hideBezier;
                break;
		}
	}
	if(action == GLFW_REPEAT) {
        if(key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL){
          //  holdingControl = true;
        }
        ring = true;
        switch (key) {
            case GLFW_KEY_W:
                lookAt.x += pos * cos(glm::radians(turnHead));
                lookAt.z -= pos * sin(glm::radians(turnHead));
                checkPos();
                break;
            case GLFW_KEY_S:
                lookAt.x -= pos * cos(glm::radians(turnHead));
                lookAt.z += pos * sin(glm::radians(turnHead));
                checkPos();
                break;
            case GLFW_KEY_D:
                turnHead -= pos * 4;
                break;
            case GLFW_KEY_A:
                turnHead += pos * 4;
                break;
            case GLFW_KEY_LEFT_SHIFT:
                lookAt.y -= pos;
                checkPos();
                break;
            case GLFW_KEY_SPACE:
                lookAt.y += pos;
                checkPos();
                break;
        }
    }
	if(action == GLFW_RELEASE){
        //ring = false;
       // ringPos = 1.0;
      //  ringPos2 = 0.0;
      //  holdingControl = false;
    }

}


// cursor_callback() ///////////////////////////////////////////////////////////
//
//  GLFW callback for mouse movement. We update cameraPhi and/or cameraTheta
//      based on how much the user has moved the mouse in the
//      X or Y directions (in screen space) and whether they have held down
//      the left or right mouse buttons. If the user hasn't held down any
//      buttons, the function just updates the last seen mouse X and Y coords.
//
////////////////////////////////////////////////////////////////////////////////

static void cursor_callback( GLFWwindow *window, double x, double y ) {
    if(leftMouseButton == GLFW_PRESS && holdingControl == true){
        float changeX = x - mousePos.x;
        float changeY = mousePos.y - y;
        int signX;
        int signY;
        if(x > 320) {
            signX = (changeX / abs(changeX));
        }else{
            signX = (-changeX / abs(changeX));
        }
        if(y > 240){
            signY = (changeY/abs(changeY));
        }else{
            signY = (-changeY/abs(changeY));
        }
        float totalChange = sqrt(pow(changeX,2)+pow(changeY,2));
        if(signX <= 0 && signY >= 0){
            scale -= totalChange*0.005;
        }else if(signY < 0 && signX > 0){
            scale += totalChange*0.005;
        }


    }
    else if( leftMouseButton == GLFW_PRESS ) {
        float changeX = x - mousePos.x;
        cameraTheta += changeX * 0.005;


        float changeY = mousePos.y - y;
        cameraPhi += changeY * 0.005;
        if(cameraPhi <= glm::radians(1.0f)){
            cameraPhi = glm::radians(1.0f);
        }else if(cameraPhi >= M_PI-glm::radians(1.0f)){
            cameraPhi = M_PI-glm::radians(1.0f);
        }
        recomputeOrientation();     // update camera (x,y,z) based on (theta,phi)
    }

    mousePos.x = x;
    mousePos.y = y;
}

// mouse_button_callback() /////////////////////////////////////////////////////
//
//  GLFW callback for mouse clicks. We save the state of the mouse button
//      when this is called so that we can check the status of the mouse
//      buttons inside the motion callback (whether they are up or down).
//
////////////////////////////////////////////////////////////////////////////////
static void mouse_button_callback( GLFWwindow *window, int button, int action, int mods ) {
	if( button == GLFW_MOUSE_BUTTON_LEFT ) {
		leftMouseButton = action;
	}
}

//*************************************************************************************
//
// Rendering / Drawing Functions - this is where the magic happens!

// drawGrid() //////////////////////////////////////////////////////////////////
//
//  Function to draw a grid in the XZ-Plane using OpenGL 2D Primitives (GL_LINES)
//
////////////////////////////////////////////////////////////////////////////////
void drawGrid() {
	/*
     *	We will get to why we need to do this when we talk about lighting,
     *	but for now whenever we want to draw something with an OpenGL
     *	Primitive - like a line, triangle, point - we need to disable lighting
     *	and then reenable it for use with the CSCI441 3D Objects.
     */
	glDisable( GL_LIGHTING );

	/** TODO #3: DRAW A GRID IN THE XZ-PLANE USING GL_LINES **/
	glColor3f(1,1,1);
	for(int i = -50; i <= 50; i++){
	    for(int j = -50; j <= 50; j++){
            glBegin(GL_LINES);{
                glVertex3f(i,0,j);
                glVertex3f(i,0,j+1);
            }
            glEnd();
            glBegin(GL_LINES);{
                glVertex3f(j,0,i);
                glVertex3f(j+1,0,i);
            }
            glEnd();
	    }

	}

	/*
     *	As noted above, we are done drawing with OpenGL Primitives, so we
     *	must turn lighting back on.
     */
	glEnable( GL_LIGHTING );
}

// drawCity() //////////////////////////////////////////////////////////////////
//
//  Function to draw a random city using CSCI441 3D Cubes
//
////////////////////////////////////////////////////////////////////////////////


void drawCity() {
	// translate up based on surface 
    glColor3f(1,1,1);

    for(int i = -50; i <= 50; i++){
        for(int j = -50; j <= 50; j++) {
            float rnd = getRand();
            if(i%2 == 0 && j%2 == 0 && rnd < 0.2){
                float height = getRand()*10 + 1;
                
				float main_c, lower_c;	//main color is highest hue, lower < main
				float rng = .4;		//color range within rgb values

				main_c = .6;		//high saturation
				lower_c = .4;		//mid- light range   
				
				//variations of main_color (in this case, green mostly)
				glColor3f(rng*getRand() + lower_c, rng*getRand() + main_c, rng * getRand() + lower_c); 
                
				glm::mat4 transMtx = glm::translate(glm::mat4(1.0f), glm::vec3(float(i),height/2,float(j)));
                glm::mat4 scaleMtx = glm::scale(glm::mat4(1.0f), glm::vec3(1.0,height,1.0));
                glMultMatrixf(&(transMtx)[0][0]);
                glMultMatrixf(&(scaleMtx)[0][0]);
                CSCI441::drawSolidCube(1);
                glMultMatrixf(&(glm::inverse(scaleMtx))[0][0]);
                glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
            }
        }

    }
}

void drawOuterRing(){
    glColor3f(166.0/255.0, 77.0/255.0, 1);
    glm::mat4 rotateClockwiseMtx = glm::rotate(glm::mat4(1.0f), glm::radians(rotate), glm::vec3(0.0f,1.0f,0.0f));
    glMultMatrixf(&rotateClockwiseMtx[0][0]);
    glm::mat4 rotateMtx = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f,0.0f,0.0f));
    glMultMatrixf(&rotateMtx[0][0]);
    CSCI441::drawSolidTorus(0.5,1.25,15,15);
    glMultMatrixf(&(glm::inverse(rotateMtx))[0][0]);
    glMultMatrixf(&(glm::inverse(rotateClockwiseMtx))[0][0]);
}


void drawBody(){
    glColor3f(102.0/255.0, 1, 1);
    glm::mat4 transMtx = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f,0.25f,0.0f));
    glMultMatrixf(&transMtx[0][0]);

    CSCI441::drawSolidSphere(1, 15, 15);
    glMultMatrixf(&(glm::inverse(transMtx))[0][0]);

    glColor3f(166.0/255.0, 77.0/255.0, 1);
    transMtx = glm::translate(glm::mat4(1.0f), glm::vec3(0.4f,0.6f,0.0f));
    glMultMatrixf(&transMtx[0][0]);
    CSCI441::drawSolidCylinder(0.7, 0.15, 0.5, 15, 15);
    glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
}

void drawVehicle(){
    //Draws a spaceship

    drawOuterRing();

    drawBody();

    glm::mat4 rotateCounterClockwiseMtx = glm::rotate(glm::mat4(1.0f), glm::radians(-rotate), glm::vec3(0.0f,1.0f,0.0f));

    glColor3f(102.0/255.0, 1, 1);


    glMultMatrixf(&rotateCounterClockwiseMtx[0][0]);
    CSCI441::drawSolidCone(0.5, 2, 15, 15);

    glColor3f(166.0/255.0, 77.0/255.0, 1);
    glm::mat4 transMtx = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f,1.98f,0.0f));
    glMultMatrixf(&transMtx[0][0]);
    CSCI441::drawSolidSphere(0.10, 15, 15);
    glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
    glMultMatrixf(&(glm::inverse(rotateCounterClockwiseMtx))[0][0]);



    rotate++;
}

// generateEnvironmentDL() /////////////////////////////////////////////////////
//
//  This function creates a display list with the code to draw a simple
//      environment for the user to navigate through.
//
//  And yes, it uses a global variable for the display list.
//  I know, I know! Kids: don't try this at home. There's something to be said
//      for object-oriented programming after all.
//
////////////////////////////////////////////////////////////////////////////////
void generateEnvironmentDL() {
	// TODO #1 Create a Display List & Call our Drawing Functions
    environmentDL = glGenLists(1);
    glNewList(environmentDL, GL_COMPILE);
        drawCity();
        drawGrid();



    glEndList();
}



					//Mia's Object//


float rotAngle = 10.0; //global rotate angle

void drawWings() { //left and right


	glm::mat4 tmtx = glm::translate(glm::mat4(1.0f),
		glm::vec3(0.3, 0.5, 0.3));

	glm::mat4 rmtx = glm::rotate(glm::mat4(1.0f), sin(rotAngle),
		glm::vec3(0, 1.0, 0));

	glColor3f(0.4, 1.0, 0.4); //green

	glMultMatrixf(&tmtx[0][0]); {
		glMultMatrixf(&rmtx[0][0]); {

			CSCI441::drawWireDisk(0.0, 0.5, 8, 5);	//equally spaced wings on each side

		};glMultMatrixf(&(glm::inverse(rmtx))[0][0]);
	}; glMultMatrixf(&(glm::inverse(tmtx))[0][0]);


	tmtx = glm::translate(glm::mat4(1.0f),
		glm::vec3(-0.3, 0.5, 0.3));
	rmtx = glm::rotate(glm::mat4(1.0f), -sin(rotAngle),
		 glm::vec3(0, 1.0, 0));

	glMultMatrixf(&tmtx[0][0]); {
		glMultMatrixf(&rmtx[0][0]); {

		glColor3f(0.4, 1.0, 0.4); //green
			CSCI441::drawWireDisk(0.0, 0.5, 8, 5);	//equally spaced wings on each side

		}; glMultMatrixf(&(glm::inverse(rmtx))[0][0]);
	}; glMultMatrixf(&(glm::inverse(tmtx))[0][0]);


	rotAngle+= .3;
}

//drawBody()
//mysterious winged thing
void M_drawBody() {

	glColor3f(0.5, 0.5, 1.0); 
	//light blue
	CSCI441::drawSolidCone(0.5, 1.3, 3, 5);

}


// drawCar() /////////////////////////
//it has flapping wings

float origX =0.0;  //global carspot within bounds- radius
float origZ =0.0;
float origY = 1.0;	//idle up& down motion


void drawCar() {


	//bob loop
	glm::mat4 carspot = glm::translate(glm::mat4(1.0f),
		glm::vec3(origX, origY, origZ)); //indicates where car is located (variable)

	glMultMatrixf(&carspot[0][0]); {

		//flap loop
		drawWings();
		//flap loop

		M_drawBody();
		//bob loop

	}; glMultMatrixf(&(glm::inverse(carspot))[0][0]);


}

// End M Object //










//
//	void renderScene()
//
//		This method will contain all of the objects to be drawn.
//




void drawCalvin(){
    glColor3f(0,1,1);
    CSCI441::drawSolidSphere(0.3,15,15);



    glm::mat4 rotateMtx = glm::rotate(glm::mat4(1.0f), glm::radians(30.0f), glm::vec3(0.0f,1.0f,0.0f));
    glMultMatrixf(&rotateMtx[0][0]);
    
    glMultMatrixf(&(glm::inverse(rotateMtx))[0][0]);

    rotateMtx = glm::rotate(glm::mat4(1.0f), glm::radians(-30.0f), glm::vec3(0.0f,1.0f,0.0f));
    glMultMatrixf(&rotateMtx[0][0]);
   
    glMultMatrixf(&(glm::inverse(rotateMtx))[0][0]);



}




void renderScene(void)  {
	// TODO #2: REMOVE TEAPOT & CREATE A CITY SCENE ON A GRID...but call it's display list!
	glCallList(environmentDL);
    glm::mat4 rotateMtx = glm::rotate(glm::mat4(1.0f), glm::radians(turnHead), glm::vec3(0.0f,1.0f,0.0f));
    glm::mat4 transMtx = glm::translate(glm::mat4(1.0f), glm::vec3(lookAt.x,lookAt.y,lookAt.z));

    glMultMatrixf(&transMtx[0][0]);
    glMultMatrixf(&rotateMtx[0][0]);


	drawCar();  

	   
    glColor3f(0.0, 1.0, 0.0);
    for(int i = 0; i < controlPoints.size(); i++) {
        glm::mat4 transMtx = glm::translate(glm::mat4(1.0f),controlPoints.at(i));
        glMultMatrixf(&transMtx[0][0]);
        if(!hideControlCage) {
            CSCI441::drawSolidSphere(0.1, 5, 5);
        }
        glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
    }


	glColor3f(0.6, 0.0, 0.5);
	for (int j = 0; i < controlSurf.size(); j++) {
		glm::mat4 transMtx = glm::translate(glm::mat4(1.0f), controlSurf.at(j));
		glMultMatrixf(&transMtx[0][0]);
		if (!hideControlCage) {
			CSCI441::drawSolidSphere(0.1, 5, 5);
		}
		glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
	}

	
	
	glDisable( GL_LIGHTING );
    glColor3f( 1, 1, 0 );
    glBegin(GL_LINE_STRIP);
    glLineWidth(3.0f);
    for(int i = 0; i < controlPoints.size(); i++) {
        if(!hideControlCage) {
            glVertex3f(controlPoints.at(i).x, controlPoints.at(i).y, controlPoints.at(i).z);
        }
    }
    glEnd();
    glEnable( GL_LIGHTING );
  


    points.clear();
    for(int i = 0; i < controlPoints.size()-3; i+=3) {
        if(c1 && i > 0){
            glm::vec3 point2 = glm::vec3(controlPoints.at(i) + (controlPoints.at(i) - controlPoints.at(i-1)));
            renderBezierCurve(controlPoints.at(i), point2, controlPoints.at(i + 2), controlPoints.at(i + 3), resolution);
        }else {
            renderBezierCurve(controlPoints.at(i), controlPoints.at(i + 1), controlPoints.at(i + 2), controlPoints.at(i + 3), resolution);
        }
    }
    if(controlPoints.at(0) != controlPoints.at(controlPoints.size()-1) && c1) {
        glm::vec3 extraPoint1 = glm::vec3(controlPoints.at(controlPoints.size()-1) + (controlPoints.at(controlPoints.size()-1) - controlPoints.at(controlPoints.size()-2)));
        glm::vec3 extraPoint2 = glm::vec3(controlPoints.at(0) + (controlPoints.at(0) - controlPoints.at(1)));
        renderBezierCurve(controlPoints.at(controlPoints.size()-1), extraPoint1, extraPoint2, controlPoints.at(0), resolution);

    }else if(controlPoints.at(0) != controlPoints.at(controlPoints.size()-1)){
        glm::vec3 extraPoint1 = glm::vec3(controlPoints.at(controlPoints.size()-1).x, controlPoints.at(controlPoints.size()-1).y-5, controlPoints.at(controlPoints.size()-1).z);
        glm::vec3 extraPoint2 = glm::vec3(controlPoints.at(0).x, controlPoints.at(0).y-5, controlPoints.at(0).z);
        renderBezierCurve(controlPoints.at(controlPoints.size()-1), extraPoint1, extraPoint2, controlPoints.at(0), resolution);
    }

	Surfpoints.clear();// DO YOU EVEN SURFACE, BRO??
	for (int i = 0; i < controlSurf.size() - 3; i += 3) {
		if (c1 && i > 0) {
			glm::vec3 point2 = glm::vec3(controlSurf.at(i) + (controlSurf.at(i) - controlSurf.at(i - 1)));
			renderBezierSurf(controlSurf.at(i), point2, controlSurf.at(i + 2), controlSurf.at(i + 3), resolution);
		}
		else {
			renderBezierSurf(controlSurf.at(i), controlSurf.at(i + 1), controlSurf.at(i + 2), controlSurf.at(i + 3), resolution);
		}
	}
	if (controlSurf.at(0) != controlSurf.at(controlSurf.size() - 1) && c1) {
		glm::vec3 extraPoint1 = glm::vec3(controlSurf.at(controlSurf.size() - 1) + (controlSurf.at(controlSurf.size() - 1) - controlSurf.at(controlSurf.size() - 2)));
		glm::vec3 extraPoint2 = glm::vec3(controlSurf.at(0) + (controlSurf.at(0) - controlSurf.at(1)));
		renderBezierSurf(controlSurf.at(controlSurf.size() - 1), extraPoint1, extraPoint2, controlSurf.at(0), resolution);

	}
	else if (controlSurf.at(0) != controlSurf.at(controlSurf.size() - 1)) {
		glm::vec3 extraPoint1 = glm::vec3(controlSurf.at(controlSurf.size() - 1).x, controlSurf.at(controlSurf.size() - 1).y - 5, controlSurf.at(controlSurf.size() - 1).z);
		glm::vec3 extraPoint2 = glm::vec3(controlSurf.at(0).x, controlSurf.at(0).y - 5, controlSurf.at(0).z);
		renderBezierSurf(controlSurf.at(controlSurf.size() - 1), extraPoint1, extraPoint2, controlSurf.at(0), resolution);
	}








    glm::mat4 transMtxTemp = glm::translate(glm::mat4(1.0f), glm::vec3(points.at(int(ridePos))));
    glMultMatrixf(&transMtxTemp[0][0]);
    drawCalvin();
    glMultMatrixf(&(glm::inverse(transMtxTemp))[0][0]);
    ridePos += 0.3;
    if(ridePos >= points.size()){
        ridePos = 0;
    }

    glMultMatrixf(&(glm::inverse(rotateMtx))[0][0]);
    glMultMatrixf(&(glm::inverse(transMtx))[0][0]);
}




//*************************************************************************************
//
// Setup Functions

//
//  void setupGLFW()
//
//      Used to setup everything GLFW related.  This includes the OpenGL context
//	and our window.
//
GLFWwindow* setupGLFW() {
	// set what function to use when registering errors
	// this is the ONLY GLFW function that can be called BEFORE GLFW is initialized
	// all other GLFW calls must be performed after GLFW has been initialized
	glfwSetErrorCallback( error_callback );

	// initialize GLFW
	if( !glfwInit() ) {
		fprintf( stderr, "[ERROR]: Could not initialize GLFW\n" );
		exit( EXIT_FAILURE );
	} else {
		fprintf( stdout, "[INFO]: GLFW initialized\n" );
	}

	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 2 );	// request OpenGL v2.X
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 1 );	// request OpenGL v2.1
	glfwWindowHint( GLFW_RESIZABLE, GLFW_FALSE );		// do not allow our window to be able to be resized

	// create a window for a given size, with a given title
	GLFWwindow *window = glfwCreateWindow( windowWidth, windowHeight, "Flight Simulator v 0.31", NULL, NULL );
	if( !window ) {						// if the window could not be created, NULL is returned
		fprintf( stderr, "[ERROR]: GLFW Window could not be created\n" );
		glfwTerminate();
		exit( EXIT_FAILURE );
	} else {
		fprintf( stdout, "[INFO]: GLFW Window created\n" );
	}

	glfwMakeContextCurrent(window);		// make the created window the current window
	glfwSwapInterval(1);				     	// update our screen after at least 1 screen refresh

	glfwSetKeyCallback( window, keyboard_callback );							// set our keyboard callback function
	glfwSetCursorPosCallback( window, cursor_callback );					// set our cursor position callback function
	glfwSetMouseButtonCallback( window, mouse_button_callback );	// set our mouse button callback function

	return window;						       // return the window that was created
}

//
//  void setupOpenGL()
//
//      Used to setup everything OpenGL related.  For now, the only setting
//	we need is what color to make the background of our window when we clear
//	the window.  In the future we will be adding many more settings to this
//	function.
//
void setupOpenGL() {
	// tell OpenGL to perform depth testing with the Z-Buffer to perform hidden
	//		surface removal.  We will discuss this more very soon.
	glEnable( GL_DEPTH_TEST );

	//******************************************************************
	// this is some code to enable a default light for the scene;
	// feel free to play around with this, but we won't talk about
	// lighting in OpenGL for another couple of weeks yet.
	GLfloat lightCol[4] = { 1, 1, 1, 1};
	GLfloat ambientCol[4] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lPosition[4] = { 10, 10, 10, 1 };
	glLightfv( GL_LIGHT0, GL_POSITION,lPosition );
	glLightfv( GL_LIGHT0, GL_DIFFUSE,lightCol );
	glLightfv( GL_LIGHT0, GL_AMBIENT, ambientCol );
	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );

	// tell OpenGL not to use the material system; just use whatever we
	// pass with glColor*()
	glEnable( GL_COLOR_MATERIAL );
	glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
	//******************************************************************

	// tells OpenGL to blend colors across triangles. Once lighting is
	// enabled, this means that objects will appear smoother - if your object
	// is rounded or has a smooth surface, this is probably a good idea;
	// if your object has a blocky surface, you probably want to disable this.
	glShadeModel( GL_SMOOTH );

	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );	// set the clear color to black
}

//
//  void setupScene()
//
//      Used to setup everything scene related.  Give our camera an
//	initial starting point and generate the display list for our city
//
void setupScene() {
	// give the camera a scenic starting point.
	camPos.x = 60;
	camPos.y = 40;
	camPos.z = 30;
	lookAt.x = 0;
	lookAt.y = 0;
	lookAt.z = 0;
	cameraTheta = M_PI / 3.0f;
	cameraPhi = M_PI / 2.8f;
	recomputeOrientation();

	srand( time(NULL) );	// seed our random number generator
	generateEnvironmentDL();
}

///*************************************************************************************
//
// Our main function

//
//	int main( int argc, char *argv[] )
//
//		Really you should know what this is by now.  We will make use of the parameters later
//
string filename;
int main( int argc, char *argv[] ) {
	// GLFW sets up our OpenGL context so must be done first
	GLFWwindow *window = setupGLFW();	// initialize all of the GLFW specific information releated to OpenGL and our window
	setupOpenGL();										// initialize all of the OpenGL specific information
	setupScene();											// initialize objects in our scene


    // prompt coaster file

    cout << "Enter a filename: ";
    cin >> filename;

    loadControlPoints(filename);



	//default to 2d grid otherwise

	loadSurfPoints("Bez_surf.csv");

	//  This is our draw loop - all rendering is done here.  We use a loop to keep the window open
	//	until the user decides to close the window and quit the program.  Without a loop, the
	//	window will display once and then the program exits.
	while( !glfwWindowShouldClose(window) ) {	// check if the window was instructed to be closed
		glDrawBuffer( GL_BACK );				// work with our back frame buffer
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );	// clear the current color contents and depth buffer in the window

		// update the projection matrix based on the window size
		// the GL_PROJECTION matrix governs properties of the view coordinates;
		// i.e. what gets seen - use a perspective projection that ranges
		// with a FOV of 45 degrees, for our current aspect ratio, and Z ranges from [0.001, 1000].
		glm::mat4 projMtx = glm::perspective( 45.0f, (GLfloat)windowWidth / (GLfloat)windowHeight, 0.001f, 1000.0f );
		glMatrixMode( GL_PROJECTION );	// change to the Projection matrix
		glLoadIdentity();				// set the matrix to be the identity
		glMultMatrixf( &projMtx[0][0] );// load our orthographic projection matrix into OpenGL's projection matrix state

		// Get the size of our framebuffer.  Ideally this should be the same dimensions as our window, but
		// when using a Retina display the actual window can be larger than the requested window.  Therefore
		// query what the actual size of the window we are rendering to is.
		GLint framebufferWidth, framebufferHeight;
		glfwGetFramebufferSize( window, &framebufferWidth, &framebufferHeight );

		// update the viewport - tell OpenGL we want to render to the whole window
		glViewport( 0, 0, framebufferWidth, framebufferHeight );

		glMatrixMode( GL_MODELVIEW );	// make the ModelView matrix current to be modified by any transformations
		glLoadIdentity();							// set the matrix to be the identity

		// set up our look at matrix to position our camera
		// TODO #6: Change how our lookAt matrix gets constructed
		glm::mat4 viewMtx = glm::lookAt( lookAt + camDir*scale,		// camera is located at (10, 10, 10)
										 lookAt,		// camera is looking at (0, 0, 0,)
										 glm::vec3(  0,  1,  0 ) );		// up vector is (0, 1, 0) - positive Y
		// multiply by the look at matrix - this is the same as our view martix
		glMultMatrixf( &viewMtx[0][0] );

		renderScene();					// draw everything to the window

		glfwSwapBuffers(window);// flush the OpenGL commands and make sure they get rendered!
		glfwPollEvents();				// check for any events and signal to redraw screen

        trackPointVal += 0.01f;
        if( trackPointVal > numSegments )
            trackPointVal = 0.0f;
	}

	glfwDestroyWindow( window );// clean up and close our window
	glfwTerminate();						// shut down GLFW to clean up our context

	return EXIT_SUCCESS;				// exit our program successfully!
}
