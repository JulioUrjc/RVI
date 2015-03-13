
#include "face_tracker.h"

#include <fstream>
#include <sstream>

using namespace cv;

#define CV_DEBUG

FaceTracker::FaceTracker(): done(false)
{

}

void FaceTracker::run()
{

   init();

   while (!done)
   {
      TheVideoCapturer.grab();
        
      TheVideoCapturer.retrieve( TheInputImage);

     TheInputImage.copyTo(TheInputImageFace);

        cvtColor(TheInputImageFace, grayFrames, CV_BGR2GRAY);
        equalizeHist(grayFrames, grayFrames);

        faceCade.detectMultiScale(grayFrames, faces, 1.1, 2, 0, Size(160, 160));

        if (faces.size())
        {
            unsigned int i = 0;
            Mat faceROI = grayFrames(faces[i]);

            rectangle(TheInputImageFace, Rect(faces[i].x - 25,faces[i].y - 25,faces[i].width + 35 ,faces[i].height + 35),  Scalar(0, 0, 255), 1, 1, 0);

            face_pos = osg::Vec2((faces[i].x + faces[i].width * 0.5)/grayFrames.cols, (faces[i].y + faces[i].height * 0.5)/grayFrames.rows);

         }
        
        #ifdef CV_DEBUG
         imshow("Face Detector Window", TheInputImageFace);
       
      #endif

      #ifdef CV_DEBUG
         cv::waitKey(30);
      #else
         OpenThreads::Thread::microSleep(30000); // camera is only 30 fps anyway
      #endif

   }
    
}

bool FaceTracker::init()
{
   try
   {
      TheVideoCapturer.open(2);
    
	   if (!TheVideoCapturer.isOpened())
	   {
           std::cout << "Could not open video" << std::endl;
           return false;
	   }

      String faceCascadeName = "../../Data/lbpcascade_frontalface.xml";
      imageIndex = 0;
      if( !faceCade.load( faceCascadeName ) ){ std::cout<<"--(!)Error loading\n"; return false; };

   } 
   catch (std::exception &ex)
   {
      std::cout<<"Exception :"<<ex.what()<<std::endl;
   }

	return true;

}

void FaceTracker::getFace2DPosition(osg::Vec2d & face_2d_position)
{
	face_2d_position = face_pos;
}


void FaceTracker::printFace2DPosition()
{
	std::cout << face_pos[0] << " " << face_pos[1] << std::endl;
}




