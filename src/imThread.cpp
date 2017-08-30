#include "imThread.h"

using namespace std;
using namespace yarp::os;

// Empty constructor
ImThread::ImThread(int period, const string &_name):RateThread(period), name(_name){}

// Initialize Variables
bool ImThread::threadInit()
{
    // Inits

    if (!imgOutPort.open("/OTFR_ROS/img:o"))     {
     printf("Problems opening img:o port\n");
     return false;
    }

    /* subscribe to ROS topics */
    if (!subs_img.topic("/multisense/left/image_rect_color")) {
           cerr<< "/multisense/left/image_rect_color" << endl;
           return -1;       }

    cout << "Img thread running" << endl;

    return true;
}

void ImThread::run()
{
    if(subs_img.getInputCount()) {
        subs_img.read(imgIn);

        cout << "Read ROS image of size " <<  imgIn.height() << " x " << imgIn.width() << endl;
        typeIm &imgOut  = imgOutPort.prepare();
        imgOut = imgIn;
        imgOutPort.write();
    } else {
        cout << "No image input connected\n";
    }
    return;
}


void ImThread::threadRelease()
{
    cout << "Closing Image Thread" << endl;
    this->stop();

    return;
}
