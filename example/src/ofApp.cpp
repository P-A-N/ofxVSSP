#include "ofApp.h"

void ofApp::setup() 
{
	vssp = make_unique<ofxVSSP>("10.0.0.10", 10940, 4, 4);
}
void ofApp::exit()
{
	vssp.reset();
}
void ofApp::update() 
{
	vssp->update();
}
void ofApp::draw() 
{
	vssp->draw_points();
}
void ofApp::keyPressed(int key) {}
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}