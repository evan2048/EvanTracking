
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
cv::Mat frame;  //video frame
cv::Mat targetSelectFrame;
int waitKeyDelay = 10;
bool isEnableSelecteTarget = false;
bool isSelectingTarget = false;
cv::Point targetStartPoint;
cv::Point targetEndPoint;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	//if not select target mode,just return
	if(isEnableSelecteTarget == false)
	{
		return;
	}
	//enter select target mode
	if(event == CV_EVENT_LBUTTONDOWN && isSelectingTarget == false)
	{
		isSelectingTarget = true;
		targetStartPoint = cv::Point(x, y);
	}
	else if(event == CV_EVENT_MOUSEMOVE && isSelectingTarget == true)
	{
		targetEndPoint = cv::Point(x, y);
		cv::Mat tempFrame = targetSelectFrame.clone();
		cv::rectangle(tempFrame, targetStartPoint, targetEndPoint, CV_RGB(255, 0, 0), 1, 8, 0);
		cv::imshow(mainWindowString, tempFrame);
	}
	else if(event == CV_EVENT_LBUTTONUP && isSelectingTarget == true)
	{
		isSelectingTarget = false;
		isEnableSelecteTarget = false;
		targetEndPoint = cv::Point(x, y);
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
    //cv::Mat first;
    //cap >> first;
    //dlib::cv_image<bgr_pixel> cfirst(first);
    //correlation_tracker tracker;
    //tracker.start_track(cfirst, centered_rect(point(93,110), 38, 86));
    while(1)
    {
        cap >> frame;
        if(frame.empty())
        {
        	break;
        }
        //dlib::cv_image<bgr_pixel> dFrame(frame);
        //tracker.update(dFrame);
        //imgWindow.clear_overlay();
        //imgWindow.add_overlay(tracker.get_position());
        if(cv::waitKey(waitKeyDelay) == 's')
        {
        	isEnableSelecteTarget = true;
        	targetSelectFrame = frame.clone();
        }
        else if(cv::waitKey(waitKeyDelay) == 'q')
        {
        	break;
        }

        if(isEnableSelecteTarget==false)
        {
        	imshow(mainWindowString, frame);
        }
    }
    cap.release();
    cv::destroyWindow(mainWindowString);
}

