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


#include <OTFR_ROS_IDLServer.h>

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
    yarp::os::BufferedPort<yarp::os::Bottle> dataInPort;             // Receives pairs of action-effect information to update affordance information
    yarp::os::BufferedPort<yarp::os::Bottle> labelsInPort;          // Receives labels for affordances
    yarp::os::BufferedPort<yarp::os::Bottle> labelOutPort;            // Streams the success rate (affordances) of the active label

    /* Ports Img */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imgInPort;		// Receives disparity greyscale image --- Handled by the clas itself
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imgOutPort;	        // output image Port with info drawn over


    /* ROS related types: */
    /* creates a node called /yarp/listener */
    yarp::os::Node *node;

    /* subscribe to topic chatter */
    yarp::os::Subscriber<yarp::sig::ImageOf<yarp::sig::PixelRgb> > subscriber;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> imgIn;



    /* class variables */

    bool verb;
    bool closing;
    int activeLabel;

    std::string filepath;   // path for the memory file
    std::string filename;    // name for the memory file

    std::vector < std::string>  knownLabels;                        // maps string labels to index in affordance matrix
    std::vector < std::vector < std::vector <double> > > affHist;   // Keeps track of all effects, for all acts and all labels: affHist[labI][act].push_back(eff)
    std::vector < std::vector < double> > knownAffsVar;             // Saves the variance of the learned affordances, to allow for active learning (higher variance means less certainity about action outcome).

    /* functions*/

    // Private Functions

    // Helper functions
    double                      vecAvg(const std::vector<double>& vec);

public:

    // RPC Accesible methods
    bool                        setnumact(const int numAct);
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
