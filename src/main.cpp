// Self-driving car: lane following, stop sign and traffic light detection.
// Raspberry Pi 3B+ (master) sends a 4-bit command on WiringPi GPIO 21-24 to the Arduino.

#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <chrono>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

// ---------- Image buffers ----------
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal;
Mat frameFinalDuplicate, frameFinalDuplicate1;
Mat ROILane, ROILaneEnd;
Mat frame_Stop, RoI_Stop, gray_Stop;
Mat frame_Traffic, RoI_Traffic, gray_Traffic;

// ---------- Lane variables ----------
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd;
vector<int> histrogramLane;
vector<int> histrogramLaneEnd;

// ---------- Detection variables ----------
CascadeClassifier Stop_Cascade, Traffic_Cascade;
vector<Rect> Stop, Traffic;
double dist_Stop, dist_Traffic;
bool T = true;                 // set by Traffic_detection(); not used in the shown code
stringstream ss;

RaspiCam_Cv Camera;

Point2f Source[]      = {Point2f(40,135), Point2f(360,135), Point2f(0,185), Point2f(400,185)};
Point2f Destination[] = {Point2f(100,0),  Point2f(280,0),   Point2f(100,240), Point2f(280,240)};

// ---------- Camera ----------
void Setup(int argc, char **argv, RaspiCam_Cv &Camera)
{
    Camera.set(CAP_PROP_FRAME_WIDTH,  400);
    Camera.set(CAP_PROP_FRAME_HEIGHT, 240);
    Camera.set(CAP_PROP_BRIGHTNESS,   50);
    Camera.set(CAP_PROP_CONTRAST,     50);
    Camera.set(CAP_PROP_SATURATION,   50);
    Camera.set(CAP_PROP_GAIN,         50);
    Camera.set(CAP_PROP_FPS,          0);
}

void Capture()
{
    Camera.grab();
    Camera.retrieve(frame);
    cvtColor(frame, frame_Traffic, COLOR_BGR2RGB);
    cvtColor(frame, frame, COLOR_BGR2RGB);
    // NOTE: frame_Stop is also used by Stop_detection(). It is not filled in the
    // screenshot. Add the line below, or Stop_detection() will work on an empty Mat.
    cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
}

// ---------- Lane pipeline ----------
void Perspective()
{
    line(frame, Source[0], Source[1], Scalar(0,0,255), 2);
    line(frame, Source[1], Source[3], Scalar(0,0,255), 2);
    line(frame, Source[3], Source[2], Scalar(0,0,255), 2);
    line(frame, Source[2], Source[0], Scalar(0,0,255), 2);

    Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, framePers, Matrix, Size(400,240));
}

void Threshold()
{
    cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
    inRange(frameGray, 170, 250, frameThresh);
    Canny(frameGray, frameEdge, 900, 900, 3, false);
    add(frameThresh, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
    cvtColor(frameFinal, frameFinalDuplicate,  COLOR_RGB2BGR);   // used in histogram only
    cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);   // used in histogram only
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();
    for (int i = 0; i < 400; i++)                        // frame.size().width = 400
    {
        ROILane = frameFinalDuplicate(Rect(i, 140, 1, 100));
        divide(255, ROILane, ROILane);
        histrogramLane.push_back((int)(sum(ROILane)[0]));
    }

    histrogramLaneEnd.resize(400);
    histrogramLaneEnd.clear();
    for (int i = 0; i < 400; i++)
    {
        ROILaneEnd = frameFinalDuplicate1(Rect(i, 0, 1, 240));
        divide(255, ROILaneEnd, ROILaneEnd);
        histrogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));
    }

    laneEnd = sum(histrogramLaneEnd)[0];
    cout << "Lane END = " << laneEnd << endl;
}

void LaneFinder()
{
    vector<int>::iterator LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr);

    vector<int>::iterator RightPtr = max_element(histrogramLane.begin() + 250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);

    line(frameFinal, Point2f(LeftLanePos, 0),  Point2f(LeftLanePos, 240),  Scalar(0,255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
}

void LaneCenter()
{
    laneCenter = (RightLanePos - LeftLanePos) / 2 + LeftLanePos;
    frameCenter = 188;

    line(frameFinal, Point2f(laneCenter, 0),  Point2f(laneCenter, 240),  Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255,0,0), 3);

    Result = laneCenter - frameCenter;
}

// ---------- Stop sign detection (Haar cascade) ----------
void Stop_detection()
{
    if (!Stop_Cascade.load("//home/pi/Desktop/Stop_cascade.xml"))
    {
        printf("Unable to open stop cascade file");
    }

    RoI_Stop = frame_Stop(Rect(0, 0, 400, 240));
    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop, Stop);

    for (int i = 0; i < Stop.size(); i++)
    {
        Point P1(Stop[i].x, Stop[i].y);
        Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);

        rectangle(RoI_Stop, P1, P2, Scalar(0,0,255), 2);
        putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255,255), 2);
        dist_Stop = (-0.32) * (P2.x - P1.x) + 35;

        ss.str(" ");
        ss.clear();
        ss << "D = " << dist_Stop << "cm";
        putText(RoI_Stop, ss.str(), Point2f(1,130), 0, 1, Scalar(0,0,255), 2);
    }
}

// ---------- Traffic light detection (Haar cascade) ----------
void Traffic_detection()
{
    if (!Traffic_Cascade.load("//home/pi/Desktop/cascade-TL.xml"))
    {
        printf("Unable to open traffic cascade file");
    }

    RoI_Traffic = frame_Traffic(Rect(0, 0, 400, 240));
    cvtColor(RoI_Traffic, gray_Traffic, COLOR_RGB2GRAY);
    equalizeHist(gray_Traffic, gray_Traffic);
    Traffic_Cascade.detectMultiScale(gray_Traffic, Traffic);

    for (int i = 0; i < Traffic.size(); i++)
    {
        Point P1(Traffic[i].x, Traffic[i].y);
        Point P2(Traffic[i].x + Traffic[i].width, Traffic[i].y + Traffic[i].height);

        rectangle(RoI_Traffic, P1, P2, Scalar(0,0,255), 2);
        putText(RoI_Traffic, "Traffic Light", P1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255,255), 2);
        dist_Traffic = (-1.07) * (P2.x - P1.x) + 80.27;

        // Always false, because P2.x > P1.x. Kept from the screenshot.
        if (P1.x - P2.x > 0)
        {
            T = false;
        }

        ss.str(" ");
        ss.clear();
        ss << "D = " << dist_Traffic << "cm";
        putText(RoI_Traffic, ss.str(), Point2f(1,130), 0, 1, Scalar(0,0,255), 2);
    }
}

// ---------- Main loop ----------
// NOTE: main() was not in the screenshots. This is reconstructed from the earlier
// version and from the Arduino command table. Check it against your original.
int main(int argc, char **argv)
{
    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);

    Setup(argc, argv, Camera);
    cout << "Connecting to camera" << endl;
    if (!Camera.open())
    {
        cout << "Failed to Connect" << endl;
        return -1;
    }
    cout << "Camera Id = " << Camera.getId() << endl;

    while (1)
    {
        auto start = chrono::system_clock::now();

        Capture();
        Perspective();
        Threshold();
        Histrogram();
        LaneFinder();
        LaneCenter();
        Stop_detection();
        Traffic_detection();

        // Stop sign in range: command 11
        if (dist_Stop > 5 && dist_Stop < 20)
        {
            digitalWrite(21, 1); digitalWrite(22, 1); digitalWrite(23, 0); digitalWrite(24, 1); // 11
            cout << "Stop Sign" << endl;
            dist_Stop = 0;
            goto Stop_Sign;
        }

        // Red light in range: command 7
        if (dist_Traffic > 5 && dist_Traffic < 20)
        {
            digitalWrite(21, 1); digitalWrite(22, 1); digitalWrite(23, 1); digitalWrite(24, 0); // 7
            cout << "Traffic light RED" << endl;
        }

        // Steering
        if (Result == 0)
        {
            digitalWrite(21, 0); digitalWrite(22, 0); digitalWrite(23, 0); digitalWrite(24, 0); // 0
            cout << "Forward" << endl;
        }
        else if (Result > 0 && Result < 10)
        {
            digitalWrite(21, 1); digitalWrite(22, 0); digitalWrite(23, 0); digitalWrite(24, 0); // 1
            cout << "Right1" << endl;
        }
        else if (Result >= 10 && Result < 20)
        {
            digitalWrite(21, 0); digitalWrite(22, 1); digitalWrite(23, 0); digitalWrite(24, 0); // 2
            cout << "Right2" << endl;
        }
        else if (Result > 20)
        {
            digitalWrite(21, 1); digitalWrite(22, 1); digitalWrite(23, 0); digitalWrite(24, 0); // 3
            cout << "Right3" << endl;
        }
        else if (Result < 0 && Result > -10)
        {
            digitalWrite(21, 0); digitalWrite(22, 0); digitalWrite(23, 1); digitalWrite(24, 0); // 4
            cout << "Left1" << endl;
        }
        else if (Result <= -10 && Result > -20)
        {
            digitalWrite(21, 1); digitalWrite(22, 0); digitalWrite(23, 1); digitalWrite(24, 0); // 5
            cout << "Left2" << endl;
        }
        else if (Result < -20)
        {
            digitalWrite(21, 0); digitalWrite(22, 1); digitalWrite(23, 1); digitalWrite(24, 0); // 6
            cout << "Left3" << endl;
        }

    Stop_Sign:
        ss.str(" ");
        ss.clear();
        if (Result == 0)     ss << "Result = " << Result << " Move Forward";
        else if (Result > 0) ss << "Result = " << Result << " Move Right";
        else                 ss << "Result = " << Result << " Move Left";
        putText(frame, ss.str(), Point2f(1,50), 0, 1, Scalar(0,0,255), 2);

        namedWindow("orignal", WINDOW_KEEPRATIO);
        moveWindow("orignal", 0, 100);
        resizeWindow("orignal", 640, 480);
        imshow("orignal", frame);

        namedWindow("Perspective", WINDOW_KEEPRATIO);
        moveWindow("Perspective", 640, 100);
        resizeWindow("Perspective", 640, 480);
        imshow("Perspective", framePers);

        namedWindow("Final", WINDOW_KEEPRATIO);
        moveWindow("Final", 1280, 100);
        resizeWindow("Final", 640, 480);
        imshow("Final", frameFinal);

        waitKey(1);

        auto end = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds = end - start;
        float t = elapsed_seconds.count();
        int FPS = 1 / t;
        // cout << "FPS = " << FPS << endl;
    }

    return 0;
}