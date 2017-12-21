#include <iostream>
#include "MyFreenectDevice.hpp"
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpParseArgv.h>


int main(int argc, const char ** argv){
    std::string opt_ipath;
    std::string configFile;
    std::string modelFile;
    std::string initFile;

    opt_ipath = "/home/mscv/Desktop/VisualServoingPractical2/VirtualVisualServoingPractical_Wireframe";
    configFile = opt_ipath + vpIoTools::path("/configFiles/box.xml");
    modelFile = opt_ipath + vpIoTools::path("/configFiles/box.cao");
    initFile = opt_ipath + vpIoTools::path("/configFiles/box");


    vpImage<unsigned char>  I(480,640);
    vpDisplayX d(I);
    d.init(I, 100, 100, "3D model Tracking") ;
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.startVideo();

    //Initialize the aquisition (just grabb some images from the kinect)
    for(int i=0;i<50;i++){
        device.getVideo(I);
        vpDisplay::display(I);
        vpDisplay::flush(I);
    }

    //*******************************************
    //*************TODO by STUDENTS**************
    //*******************************************
    vpHomogeneousMatrix cMo;

    vpMbEdgeTracker tracker;

    // Load tracker config file
    tracker.loadConfigFile(configFile);

    // initialise vpCameraParameters from the tracker
    vpCameraParameters cam;
    tracker.getCameraParameters(cam);

    // Load the 3D model
    tracker.loadModel(modelFile);

    //Initialize the tracking.
    tracker.initClick(I, initFile, true);

    //track the model
    tracker.track(I);
    tracker.getPose(cMo);
    tracker.display(I, cMo, cam, vpColor::red, 1);
    vpDisplay::flush(I);


    while (true){

        device.getVideo(I);
        vpDisplay::display(I);

        //track the model
        tracker.track(I);
        tracker.getPose(cMo);
        //to display the  tracked 3D model
        tracker.display(I, cMo, cam, vpColor::green, 1);
        //to display the coordinate frame
        vpDisplay::displayFrame (I, cMo, cam, 0.05, vpColor::blue);
        vpDisplay::flush(I);


    }


    return 0;
}

