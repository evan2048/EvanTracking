/*
*2015.9.3 use 4 space instead tab
*         add reSelect target feature
*2015.9.4 add ros support
*/
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
//opencv
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace dlib;
using namespace std;
//using namespace cv;

const string mainWindowString="EvanTracking";
int waitKeyDelay = 10;
bool isSelectingTarget = false;
bool isMouseLeftButtonDown = false;
bool isTracking = false;
bool isReSelectingTarget = false;
cv::Mat frame;  //video frame
cv::Mat targetFrame;
cv::Rect targetRect;
cv::Mat targetMat;
cv::Point targetInitStartPoint;
cv::Point targetInitEndPoint;
cv::Point targetCurrentStartPoint;
cv::Point targetCurrentEndPoint;
cv::Point reSelectTartgetCenterPoint;
correlation_tracker tracker;

void swapTargetInitPosition()
{
    if((targetInitStartPoint.x > targetInitEndPoint.x) || (targetInitStartPoint.y > targetInitEndPoint.y))
    {
        cv::Point temp = targetInitStartPoint;
        targetInitStartPoint = targetInitEndPoint;
        targetInitEndPoint = temp;
    } 
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    //select target mode
    if(isSelectingTarget == true)
    {
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
            swapTargetInitPosition();  //check position
            targetRect = cv::Rect(targetInitStartPoint, targetInitEndPoint);
            targetMat = targetFrame(targetRect).clone();
            cv::namedWindow("selectedTarget", cv::WINDOW_AUTOSIZE);
            cv::moveWindow("selectedTarget", 100 + targetFrame.cols, 60);
            cv::imshow("selectedTarget", targetMat);
            //start tracking
            dlib::cv_image<bgr_pixel> dlibTarget(targetFrame);
            tracker.start_track(dlibTarget, centered_rect(point(targetRect.x + targetRect.width/2, targetRect.y + targetRect.height/2), targetRect.width, targetRect.height));
            isTracking = true;
        }
    }
    //reSelect target mode
    else if(isReSelectingTarget == true)
    {
        if(event == CV_EVENT_LBUTTONDOWN && isMouseLeftButtonDown == false)
        {
            isMouseLeftButtonDown = true;
            reSelectTartgetCenterPoint = cv::Point(x, y);
        }
        else if(event == CV_EVENT_MOUSEMOVE && isMouseLeftButtonDown == true)
        {

        }
        else if(event == CV_EVENT_LBUTTONUP && isMouseLeftButtonDown == true)
        {
            isMouseLeftButtonDown = false;
            //isReSelectingTarget = false;
            dlib::cv_image<bgr_pixel> dlibTarget(frame);
            tracker.start_track(dlibTarget, centered_rect(point(reSelectTartgetCenterPoint.x, reSelectTartgetCenterPoint.y), targetRect.width, targetRect.height));
        }
    }
}

int main(int argc, char** argv)
{
    //init ros
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nodeHandle;
    ros::Publisher tracker_pub = nodeHandle.advertise<std_msgs::String>("tracker", 1000);
    //ros::Rate loop_rate(10);  //10hz

    cv::VideoCapture cap;
    //read param
    if(argc<2)
    {
        printf("error:param error\n");
        return -1;
    }
    else if(argc==2)
    {
        cap.open(argv[1]);
    }
    cv::namedWindow(mainWindowString, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(mainWindowString, 100, 60);
    cv::setMouseCallback(mainWindowString, mouseHandler, 0);
    if(!cap.isOpened())
    {
        printf("error:VideoCapture open failed\n");
        return -1;
    }
    while(1)
    {
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
            isReSelectingTarget = true;
        }
        else if(key == 'q')
        {
            break;
        }
        //show preview
        if(isSelectingTarget == false)
        {
            cap >> frame;
            if(frame.empty())
            {
                printf("info:frame empty\n");
                break;
            }
            if(isTracking == true)
            {
                dlib::cv_image<bgr_pixel> dlibFrame(frame);
                tracker.update(dlibFrame);
                drectangle dlibTargetPosition = tracker.get_position();
                targetCurrentStartPoint = cv::Point(dlibTargetPosition.left(), dlibTargetPosition.top());
                targetCurrentEndPoint = cv::Point(dlibTargetPosition.right(), dlibTargetPosition.bottom());
                cv::rectangle(frame, targetCurrentStartPoint, targetCurrentEndPoint, CV_RGB(0, 0, 255), 2, 8, 0);
                //printf("target at:%d %d %d %d\n", targetCurrentStartPoint.x, targetCurrentStartPoint.y, targetCurrentEndPoint.x - targetCurrentStartPoint.x, targetCurrentEndPoint.y - targetCurrentStartPoint.y);
                //ros pub
                std_msgs::String msg;
                std::stringstream ss;
                ss << "" << targetCurrentStartPoint.x << " " << targetCurrentStartPoint.y << " " << targetCurrentEndPoint.x - targetCurrentStartPoint.x << " " << targetCurrentEndPoint.y - targetCurrentStartPoint.y;
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                tracker_pub.publish(msg);
                ros::spinOnce();
                //loop_rate.sleep();
            }
            cv::imshow(mainWindowString, frame);
        }
    }
    cap.release();
    cv::destroyAllWindows();
}
