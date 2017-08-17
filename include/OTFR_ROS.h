/*
 * OTFR_ROS COLLECTOR MODULE
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef __OTFR_ROS_H__
#define __OTFR_ROS_H__

// Includes
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <math.h>
#include <vector>
#include <ctime>
#include <map>

// YARP - iCub libs
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Publisher.h>

// ROS related libs
#include <OTFR_ROS_IDLServer.h>
#include <sensor_msgs_Image.h>
#include <stereo_msgs_DisparityImage.h>



/**********************************************************
    PUBLIC METHODS
/**********************************************************/

/**********************************************************/
class OTFR_ROS : public yarp::os::RFModule, public OTFR_ROS_IDLServer
{
protected:
    /* module parameters */
    std::string name;                     // string containing module name

    /* Ports RPC */
    yarp::os::RpcServer rpcInPort;                                      // port to handle incoming commands

    /* Ports data */

    typedef yarp::sig::ImageOf<yarp::sig::PixelRgb> typeIm;
    typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> typeDepth;

    /* Ports Img */
    yarp::os::BufferedPort<typeIm >                                     imgOutPort;             // outputs camera image (RGB)
    yarp::os::BufferedPort<typeDepth >                                  depthOutPort;            // outputs depth image (mono)
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >  depthOutPort_ros;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >  dispOutPort;

    /* ROS related types: */

    /* subscribe to ROS topic */
    yarp::os::Subscriber<typeIm >                       subs_img;
    yarp::os::Subscriber<typeDepth>                     subs_depth;
    yarp::os::Subscriber<sensor_msgs_Image>             subs_depth_ros;
    yarp::os::Subscriber<stereo_msgs_DisparityImage >   subs_disp;


    /* Image types to read and propagate*/
    typeIm                                      imgIn;
    typeDepth                                   depthIn;
    sensor_msgs_Image                           depthIn_ros;
    stereo_msgs_DisparityImage                  dispIn;

    /* creates a node for the image and another for the disparity */
    yarp::os::Node *node_yarp;

    bool img_flag;
    bool depth_flag;
    bool depth_ros_flag;
    bool disp_flag;



    /* class variables */

    bool verb;
    bool closing;

    /* functions*/

    // Private Functions

    // Helper functions

public:

    // RPC Accesible methods    
    bool						quit();

    // RF modules overrides
    bool						configure(yarp::os::ResourceFinder &rf);
    bool						interruptModule();
    bool						close();
    bool						updateModule();
   // void                        onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &img );
    double						getPeriod();

    // thrift connection //
    bool						attach(yarp::os::RpcServer &source);
};

#endif
//empty line to make gcc happy
