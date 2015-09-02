
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace dlib;
using namespace std;
//using namespace cv;

const string mainWindowString="EvanTracking";
int waitKeyDelay = 25;
bool isSelectingTarget = false;
bool isMouseLeftButtonDown = false;
bool isTracking = false;
cv::Mat frame;  //video frame
cv::Mat targetFrame;
cv::Rect targetRect;
cv::Mat targetMat;
cv::Point targetInitStartPoint;
cv::Point targetInitEndPoint;
cv::Point targetCurrentStartPoint;
cv::Point targetCurrentEndPoint;
correlation_tracker tracker;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	//if not in select target mode,just ignore and return
	if(isSelectingTarget == false)
	{
		return;
	}
	//enter select target mode
	if(event == CV_EVENT_LBUTTONDOWN && isMouseLeftButtonDown == false)
	{
		isMouseLeftButtonDown = true;
		targetInitStartPoint = cv::Point(x, y);
	}
	else if(event == CV_EVENT_MOUSEMOVE && isMouseLeftButtonDown == true)
	{
		targetInitEndPoint = cv::Point(x, y);
		cv::Mat tempFrame = targetFrame.clone();
		cv::rectangle(tempFrame, targetInitStartPoint, targetInitEndPoint, CV_RGB(255, 0, 0), 1, 8, 0);
		cv::imshow(mainWindowString, tempFrame);
	}
	else if(event == CV_EVENT_LBUTTONUP && isMouseLeftButtonDown == true)
	{
		isMouseLeftButtonDown = false;
		isSelectingTarget = false;
		targetInitEndPoint = cv::Point(x, y);
		targetRect = cv::Rect(targetInitStartPoint, targetInitEndPoint);
        targetMat = targetFrame(targetRect).clone();
        cv::namedWindow("selectedTarget",cv::WINDOW_AUTOSIZE);
        cv::imshow("selectedTarget",targetMat);
        //start tracking
        dlib::cv_image<bgr_pixel> dlibTarget(targetFrame);
        tracker.start_track(dlibTarget, centered_rect(point(targetRect.x + targetRect.width/2, targetRect.y + targetRect.height/2), targetRect.width, targetRect.height));
        isTracking = true;
	}
}

int main(int argc, char** argv)
{
    cv::VideoCapture cap(0);
    cv::namedWindow(mainWindowString,cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(mainWindowString, mouseHandler, 0);
    if(!cap.isOpened())
    {
        printf("error:VideoCapture open failed\n");
        return -1;
    }
    while(1)
    {
        cap >> frame;
        if(frame.empty())
        {
        	printf("frame empty\n");
        	break;
        }
        //read key
        char key = cv::waitKey(waitKeyDelay);
        if(key == 's' && isTracking == false)
        {
        	isSelectingTarget = true;
        	targetFrame = frame.clone();
        	cv::imshow(mainWindowString, targetFrame);
        }
        else if(key == 'r' && isTracking == true)
        {
        	isTracking = false;
        	dlib::cv_image<bgr_pixel> dlibTarget(targetFrame);
        	tracker.start_track(dlibTarget, centered_rect(point(targetRect.x + targetRect.width/2, targetRect.y + targetRect.height/2), targetRect.width, targetRect.height));
            isTracking = true;
        }
        else if(key == 'q')
        {
        	break;
        }
        //show preview
        if(isSelectingTarget == false)
        {
        	if(isTracking == true)
        	{
        		dlib::cv_image<bgr_pixel> dlibFrame(frame);
        	    tracker.update(dlibFrame);
        	    drectangle dlibTargetPosition = tracker.get_position();
        	    targetCurrentStartPoint = cv::Point(dlibTargetPosition.left(), dlibTargetPosition.top());
        	    targetCurrentEndPoint = cv::Point(dlibTargetPosition.right(), dlibTargetPosition.bottom());
        	    cv::rectangle(frame, targetCurrentStartPoint, targetCurrentEndPoint, CV_RGB(0, 0, 255), 2, 8, 0);
        	}
        	cv::imshow(mainWindowString, frame);
        }
    }
    cap.release();
    cv::destroyAllWindows();
}
