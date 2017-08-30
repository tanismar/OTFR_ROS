#include "dispThread.h"

using namespace std;
using namespace yarp::os;

// Empty constructor
DispThread::DispThread(int period, const string &_name):RateThread(period), name(_name){}

// Initialize Variables
bool DispThread::threadInit()
{
    // Inits

    if (!dispOutPort.open("/OTFR_ROS/disp:o"))     {
     printf("Problems opening disp:o port\n");
     return false;
    }

    /* subscribe to ROS topics */
    if (!subs_disp.topic("/multisense/left/disparity_image")) {
           cerr<< "/multisense/left/disparity_image" << endl;
           return -1;       }

    cout << "Disp thread running" << endl;

    return true;
}

void DispThread::run()
{
    if(subs_disp.getInputCount()) {
        subs_disp.read(dispIn);
        cout << "Disp read" << endl;

        typeDepth &dispOut  = dispOutPort.prepare();
        dispOut.setExternal(dispIn.image.data.data(), dispIn.image.width, dispIn.image.height);
        dispOutPort.write();
    } else {
        cout << "No disparity input connected\n";
    }
    return;
}


void DispThread::threadRelease()
{
    cout << "Closing Disp Thread" << endl;
    this->stop();

    return;
}
