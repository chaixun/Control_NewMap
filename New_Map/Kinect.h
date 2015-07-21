#ifndef KINECT_H
#define KINECT_H

#include <memory>

using namespace std;

class KINECT
{
private:
    class KINECT_STRUCT;
    std::auto_ptr<KINECT_STRUCT> pStruct;

public:
    KINECT();
    ~KINECT();
    void Capture();
    void ViewCloud();
    void Start();
    void Stop();
};

#endif // KINECT_H
