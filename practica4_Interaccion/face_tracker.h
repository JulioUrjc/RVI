

#include <opencv2/opencv.hpp>
#include <osg/PositionAttitudeTransform>
#include <OpenThreads/Thread>

#include <iostream>

using namespace cv;

class FaceTracker : public OpenThreads::Thread
{

public:

	FaceTracker::FaceTracker();
	
	virtual void run();
	
   void getFace2DPosition(osg::Vec2d & face_2d_position);
	
   void printFace2DPosition();

	void close() { done = true; }

	
private:

	bool init();

	std::string TheInputVideo;
		
   cv::VideoCapture TheVideoCapturer;
	cv::Mat TheInputImage;
	cv::Mat TheInputImageCopy;
   cv::Mat TheInputImageFace;
	
	bool done;

	osg::Vec3d tracked_pos;
   osg::Vec2d face_pos;

   CascadeClassifier faceCade;
   Mat camFrames, grayFrames;
   vector<Rect> faces;
   long imageIndex;


};