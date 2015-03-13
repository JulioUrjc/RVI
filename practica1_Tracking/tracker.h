
#include "aruco/aruco.h"
#include <opencv2/opencv.hpp>
#include <osg/PositionAttitudeTransform>
#include <OpenThreads/Thread>

#include <iostream>

using namespace cv;

class Tracker : public OpenThreads::Thread
{

public:

	Tracker::Tracker();
	
	virtual void run();
	void getPosition(osg::Vec3d & position);
	void printPosition();

	void close() { done = true; }

	static void initcvGUI();
	static void cvGUIEvents(int pos,void*);

	static int thresParam1;
    static int thresParam2;

private:

	bool init();

	std::string TheInputVideo;
	std::string TheIntrinsicFile;
	std::string TheBoardConfigFile;
	bool The3DInfoAvailable;
	float TheMarkerSize;
	cv::VideoCapture TheVideoCapturer;
	cv::Mat TheInputImage;
	cv::Mat TheInputImageCopy;
	aruco::CameraParameters TheCameraParameters;
	aruco::BoardConfiguration TheBoardConfig;
	aruco::BoardDetector TheBoardDetector;

	std::string TheOutVideoFilePath;
	cv::VideoWriter VWriter;

	pair<double,double> AvrgTime;//determines the average time required for detection

	int index;

	double scale;

	bool done;

	osg::Vec3d tracked_pos;


};