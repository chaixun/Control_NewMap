#include "Kinect.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace pcl;

class KINECT::KINECT_STRUCT
{
    friend class KINECT;
public:
    void pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    pcl::OpenNIGrabber* interface;

private:
    int frames_num;
    bool save_one;
};

KINECT::KINECT():pStruct(new KINECT_STRUCT)
{
    pStruct->interface = new pcl::OpenNIGrabber();
    pStruct->frames_num = 0;
    pStruct->save_one = false;
}

KINECT::~KINECT()
{
    pStruct->interface->stop();
    cout<<"Depth Close!"<<endl;
}

void KINECT::Start()
{
    pStruct->interface->start();
    cout<<"Depth Open!"<<endl;
}

void KINECT::Stop()
{
    pStruct->interface->stop();
    cout<<"Depth Stop!"<<endl;
}

void KINECT::Capture()
{
    pStruct->frames_num++;
    pStruct->save_one = true;
}

void KINECT::ViewCloud()
{
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ) > f =
            boost::bind(&KINECT::KINECT_STRUCT::pointcloud, pStruct.get(), _1);

    pStruct->interface->registerCallback (f);
}

void KINECT::KINECT_STRUCT::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr SensorPoint(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f transformation1;
    transformation1 << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    pcl::transformPointCloud(*cloud,*SensorPoint,transformation1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr RobotPoint(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transformation2;
    transformation2 << 0.9995, 0.0134, -0.0273, 0.0224,
            -0.0304, 0.5120, -0.8584, 0.2026 + 0.038,
            0.0025, 0.8589, 0.5122, 0.5733,
            0, 0, 0, 1;
    // 0.2026 + 0.85 + 0.038 + 0.2
    //        transformation2 << 0.9996, 0.0233, -0.0137, 0.0224,
    //                -0.0270, 0.8622, -0.5059, 0.2026 - 0.12,
    //                0, 0.5061, 0.8625, 0.5733,
    //                0, 0, 0, 1;
    pcl::transformPointCloud(*SensorPoint,*RobotPoint,transformation2);

    //Save PCD

    if(save_one)
    {
        save_one = false;
        std::stringstream out;
        out<<frames_num;
        std::string originalname = "originalcloud" + out.str() + ".pcd";
        pcl::io::savePCDFileASCII(originalname, *RobotPoint);

    }

    //Mapping GridMap

    Eigen::MatrixXf GridMap = Eigen::MatrixXf::Zero(120,120);
    Eigen::MatrixXf GridNum = Eigen::MatrixXf::Zero(120,120);
    for (size_t i = 0;i < RobotPoint->points.size(); ++i)
    {
        if(RobotPoint->points[i].x>-1.5&&RobotPoint->points[i].x<1.5&&
                RobotPoint->points[i].z>0&&RobotPoint->points[i].z<3)
        {
            int m, n;
            n = floor(RobotPoint->points[i].x/0.025) + 60;
            m = floor(RobotPoint->points[i].z/0.025);

            //Mean

            GridMap(m,n) = (GridMap(m,n)*GridNum(m,n) + RobotPoint->points[i].y)/(GridNum(m,n) + 1);
            GridNum(m,n) = GridNum(m,n) + 1;

            //Max

            //if (GridMap(m,n) < RobotPoint->points[i].y)
            //{
            //GridMap(m,n) = RobotPoint->points[i].y;
            //}
        }
    }

    std::ofstream Gridmapfile("GridMap.txt");
    if(Gridmapfile.is_open())
    {
        Gridmapfile<<GridMap<<endl;
    }

    boost::posix_time::ptime t(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::ptime base_time(t.date());
    boost::posix_time::time_duration diff = t - base_time;
    long milliseconds = diff.total_milliseconds();

    cout<<milliseconds<<endl;
}
