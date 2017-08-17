/*
 * TOOL 3D FEATURE EXTRACTOR
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email: tanis.mar@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OTFR_ROS.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/************************* RF overwrites ********************************/
/************************************************************************/
bool  OTFR_ROS::configure(ResourceFinder &rf)
{
    // add and initialize the port to send out the features via thrift.
    string name = rf.check("name",Value("OTFR_ROS")).asString().c_str();
    rf.setDefaultContext("OntHeFlyRecognition");

    // Check default .ini variables
    verb = rf.check("verbose",Value(true)).asBool();

    //open ports
    bool ret = true;
    ret = ret && imgOutPort.open(("/"+name+"/img:o").c_str());          // Streams images
    ret = ret && dispOutPort.open(("/"+name+"/disp:o").c_str());          // Streams diparity image
    ret = ret && depthOutPort.open(("/"+name+"/depth:o").c_str());          // Streams diparity image
    ret = ret && depthOutPort_ros.open(("/"+name+"/depth_ros:o").c_str());          // Streams diparity image
    if (!ret){
        printf("Problems opening ports\n");
        return false;
    }

    // open rpc ports
    bool retRPC = true;
    retRPC =  retRPC && rpcInPort.open("/"+name+"/rpc:i");
    if (!retRPC){
        printf("Problems opening ports\n");
        return false;
    }
    attach(rpcInPort);


    /* CONFIGURE ROS VARIABLES */
    img_flag = true;
    depth_flag = false;
    depth_ros_flag = false;
    disp_flag = true;


    /* creates a node called /yarp/listener */
    node_yarp = new Node("/yarp/node");

    //node.prepare("/yarp/listener");

    /* subscribe to ROS topics */

    if (img_flag == true){
    if (!subs_img.topic("/multisense/left/image_rect_color")) {
           cerr<< "/multisense/left/image_rect_color" << endl;
           return -1;       }}

    if (depth_flag == true){
    if (!subs_depth.topic("/multisense/depth")) {
           cerr<< "/multisense/depth" << endl;
           return -1;       }}

    if (depth_ros_flag == true){
    if (!subs_depth_ros.topic("/multisense/depth")) {
           cerr<< "/multisense/depth" << endl;
           return -1;       }}

    if (disp_flag == true){
    if (!subs_disp.topic("/multisense/left/disparity_image")) {
           cerr<< "/multisense/left/disparity_image" << endl;
           return -1;       }}


    /* Module rpc parameters */
    closing = false;

    cout << endl << "Configuring done."<<endl;

    return true;
}

bool  OTFR_ROS::updateModule()
{
     /* read data from the ROS topics */
    if (img_flag == true){
        subs_img.read(imgIn);
        cout << "Image pixel type " << imgIn.getPixelCode() << endl;
        cout << "Image read of width " << imgIn.width() << " and height "<< imgIn.height() << endl;

        printf("Propagating image!!\n");
        typeIm &imgOut  = imgOutPort.prepare();
        imgOut = imgIn;
        imgOutPort.write();
    }

    if (depth_flag ==true){
        subs_depth.read(depthIn);
        cout << "Depth pixel type " << depthIn.getPixelCode()<< endl;
        cout << "Depth read of width " << depthIn.width() << " and height "<< depthIn.height() << endl;

        printf("Propagating depth!!\n");
        typeDepth &depthOut  = depthOutPort.prepare();
        depthOut = depthIn;
        depthOutPort.write();
    }

    if (depth_ros_flag ==true){
        subs_depth_ros.read(depthIn_ros);
        cout << "Read depth_ros with encoding: " << depthIn_ros.encoding << endl;
        cout << "Depth_ros read of width " << depthIn_ros.width << " and height "<< depthIn_ros.height << endl;

        printf("Propagating depth_ros!!\n");
        ImageOf<PixelFloat> &depthOut_ros  = depthOutPort_ros.prepare();
        depthOut_ros.setExternal(depthIn_ros.data.data(), depthIn_ros.width, depthIn_ros.height);
        depthOutPort_ros.write();
    }

    if (disp_flag == true){
        subs_disp.read(dispIn);
        cout << "Read disp with encoding: " << dispIn.image.encoding << endl;
        cout << "Disp read of width " << dispIn.image.width<< " and height "<< dispIn.image.height << endl;

        printf("Propagating disp!!\n");
        ImageOf<PixelFloat> &dispOut  = dispOutPort.prepare();
        dispOut.setExternal(dispIn.image.data.data(), dispIn.image.width, dispIn.image.height);
        dispOutPort.write();
    }

    return !closing;
}


double  OTFR_ROS::getPeriod()
{
    return 0.2; //module periodicity (seconds)
}


bool  OTFR_ROS::interruptModule()
{
    closing = true;

//    imgInPort.interrupt();
    imgOutPort.interrupt();
    depthOutPort.interrupt();
    depthOutPort_ros.interrupt();
    dispOutPort.interrupt();


    cout << "Ports interrupted" << endl;
    return true;
}


bool  OTFR_ROS::close()
{

    imgOutPort.close();
    dispOutPort.close();
    depthOutPort.close();
    depthOutPort_ros.close();

    delete node_yarp;

    cout << "Module ports closed" << endl;
    return true;
}

/**************************** THRIFT CONTROL*********************************/
bool  OTFR_ROS::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

// RPC Accesible via trhift.
/**********************************************************/


/***************** MORE PUBLIC METHODS **************/

/**********************************************************/
bool OTFR_ROS::quit()
{
    closing = true;
    return true;
}


/**********************************************************
                    PRIVATE METHODS
/**********************************************************/

/***************** Helper Functions *************************************/

/***************** MORE PRIVATE METHOTS**************/


/************************************************************************/
/************************************************************************/
int main(int argc, char * argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    OTFR_ROS module;
    ResourceFinder rf;
    rf.setDefaultContext("OnTheFlyRecognition");
    rf.setDefaultConfigFile("OTFR_ROS.ini");
    rf.setDefault("name","OTFR_ROS");
    rf.setVerbose(true);
    rf.configure(argc, argv);

    cout<<"Configure and Start module..."<<endl;
    return module.runModule(rf);

}

