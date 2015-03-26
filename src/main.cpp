#include "ofMain.h"
#include "ofAppGlutWindow.h"
#include "xphysx.h"

class xApp : public ofBaseApp {

	public:
		ofEasyCam cam;
		int w,h;
		bool start;

		void setup(){
			ofSetFrameRate(150);
			w=ofGetScreenWidth();
			h=ofGetScreenHeight();
            initPX();
		}

		void exit(){
            shutdownPX();
		}

		void update(){
			ofSetWindowTitle(ofToString(ofGetFrameRate()));
		}

		void draw(){
			ofBackgroundGradient(255,0);

			cam.begin();
			    glEnable(GL_DEPTH_TEST);
			    glDepthMask(GL_TRUE);
			    if(start) renderPX();
			cam.end();
		}

		void keyPressed(int key){
			if(key == 'q')
                start=!start;
		}
};

int main( ){
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1280, 720, OF_WINDOW);
	ofRunApp(new xApp());
}
