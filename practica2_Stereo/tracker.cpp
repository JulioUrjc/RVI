
#include "tracker.h"

#include <fstream>
#include <sstream>

using namespace cv;
using namespace aruco;

int Tracker::thresParam1 = 8;
int Tracker::thresParam2 = 2;

#define CV_DEBUG

Tracker::Tracker(): AvrgTime(0,0), done(false)
{
   // where the config files are located
	TheIntrinsicFile = "../../data/camera_conf.yml";
	TheBoardConfigFile = "../../data/board_conf.yml";
	The3DInfoAvailable = false;

	// size of board
	TheMarkerSize=0.188;

	scale = 0.2;
	
	index = 0;
}

void Tracker::run()
{

   init();

   while (!done)
   {
      TheBoardDetector.getMarkerDetector().setThresholdParams(thresParam1,thresParam2);

      TheVideoCapturer.grab();
        
      TheVideoCapturer.retrieve( TheInputImage);
      TheInputImage.copyTo(TheInputImageCopy);
      index++; //number of images captured
      double tick = (double)getTickCount();//for checking the speed
      //Detection of the board
      float probDetect=TheBoardDetector.detect(TheInputImage);
      //chekc the speed by calculating the mean speed of all iterations
      AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
      AvrgTime.second++;
      #ifdef CV_DEBUG
         //cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

         //print marker borders
         for (unsigned int i=0;i<TheBoardDetector.getDetectedMarkers().size();i++)
         TheBoardDetector.getDetectedMarkers()[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
      #endif

      //print board
      if (TheCameraParameters.isValid())
      {
	      if ( probDetect>0.2)   
	      {
		      #ifdef CV_DEBUG
			      CvDrawingUtils::draw3dAxis( TheInputImageCopy,TheBoardDetector.getDetectedBoard(),TheCameraParameters);
		      #endif
		      cv::Mat pos = TheBoardDetector.getDetectedBoard().Tvec;
			
		      tracked_pos[0] = -pos.at<float>(0,0);
		      tracked_pos[1] = -pos.at<float>(1,0);
		      tracked_pos[2] = pos.at<float>(2,0);

		      tracked_pos *= scale;

	      }
      }

      #ifdef CV_DEBUG
         //show input with augmented information and  the thresholded image
         cv::imshow("in",TheInputImageCopy);
         cv::imshow("thres",TheBoardDetector.getMarkerDetector().getThresholdedImage());
      #endif

      #ifdef CV_DEBUG
         cv::waitKey(30);
      #else
         OpenThreads::Thread::microSleep(30000); // camera is only 30 fps anyway
      #endif
   }
    
}

bool Tracker::init()
{
   try
   {
      TheBoardConfig.readFromFile(TheBoardConfigFile);

      TheVideoCapturer.open(2);
    
	   if (!TheVideoCapturer.isOpened())
	   {
           std::cout << "Could not open video" << std::endl;
           return false;
	   }

      //read first image to get the dimensions
      TheVideoCapturer>>TheInputImage;

      //read camera parameters if passed
      TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
      TheCameraParameters.resize(TheInputImage.size());
    
      TheBoardDetector.setParams(TheBoardConfig,TheCameraParameters,TheMarkerSize);
      TheBoardDetector.getMarkerDetector().setThresholdParams(thresParam1,thresParam2);
      TheBoardDetector.getMarkerDetector().enableErosion(true);

      // init GUI
      #ifdef CV_DEBUG
      Tracker::initcvGUI();
      #endif CV_DEBUG

   } 
   catch (std::exception &ex)
   {
      cout<<"Exception :"<<ex.what()<<endl;
   }

	return true;

}

void Tracker::initcvGUI()
{
	//Create gui
   cv::namedWindow("thres",1);
   cv::namedWindow("in",1);

   cv::createTrackbar("ThresParam1", "in",&Tracker::thresParam1, 13, &Tracker::cvGUIEvents);
   cv::createTrackbar("ThresParam2", "in",&Tracker::thresParam2, 13, &Tracker::cvGUIEvents);

}


void Tracker::getPosition(osg::Vec3d & position)
{
	position = tracked_pos;
}

void Tracker::printPosition()
{
	std::cout << tracked_pos[0] << " " << tracked_pos[1] << " " << tracked_pos[2] << std::endl;
}

void Tracker::cvGUIEvents(int pos,void*)
{
    if (thresParam1<3) thresParam1=3;
    if (thresParam1%2!=1) thresParam1++;
    if (thresParam2<1) thresParam2=1;
}



