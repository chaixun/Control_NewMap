#include "Kinect.h"

#include <iostream>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

int main()
{
    time_t t = time(NULL);
    tm* local = new tm;
    char buf[26] = {0};
    localtime_r(&t, local);
    strftime(buf, 64, "%Y-%m-%d %H-%M-%S", local);
    mkdir(buf, S_IRWXU | S_IRWXG);
    chdir(buf);

    KINECT kinect1;
    kinect1.ViewCloud();
    kinect1.Start();

    while (1)
    {
        char c = getchar();
        if (c == 'c')
        {
            kinect1.Capture();
        }
    }

}
