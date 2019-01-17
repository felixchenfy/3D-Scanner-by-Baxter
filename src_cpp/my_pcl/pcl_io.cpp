
#include "my_pcl/pcl_io.h"

#include <iostream>
#include <memory>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

using namespace std;

#define DEBUG_RESULT true

namespace my_pcl
{

// -- Input / Output

bool read_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr &cloud)
{
    cloud.reset(new PointCloud<PointXYZRGB>);
    int load_res = io::loadPCDFile<PointXYZRGB>(filename, *cloud);
    if (load_res == -1) // load the file
    {
        string ERROR_MESSAGE = "Couldn't read file " + filename + "\n";
        PCL_ERROR(ERROR_MESSAGE.c_str());
        assert(0);
    }
    if (DEBUG_RESULT)
    {
        std::cout << "Loaded " << cloud->width <<"x"<< cloud->height << " data points from " 
        << filename << std::endl;
    }
    return true;
}


bool read_point_cloud(string filename, PointCloud<PointXYZ>::Ptr &cloud)
{
    cloud.reset(new PointCloud<PointXYZ>);
    int load_res = io::loadPCDFile<PointXYZ>(filename, *cloud);
    if (load_res == -1) // load the file
    {
        string ERROR_MESSAGE = "Couldn't read file " + filename + "\n";
        PCL_ERROR(ERROR_MESSAGE.c_str());
        assert(0);
    }
    if (DEBUG_RESULT)
    {
        std::cout << "Loaded " << cloud->width <<"x"<< cloud->height << " data points from " 
        << filename << std::endl;
    }
    return true;
}


PointCloud<PointXYZRGB>::Ptr read_point_cloud(string filename)
{
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    int load_res = io::loadPCDFile<PointXYZRGB>(filename, *cloud);
    if (load_res == -1) // load the file
    {
        string ERROR_MESSAGE = "Couldn't read file " + filename + "\n";
        PCL_ERROR(ERROR_MESSAGE.c_str());
        assert(0);
    }
    if (DEBUG_RESULT)
    {
        std::cout << "Loaded " << cloud->width <<"x"<< cloud->height << " data points from " 
        << filename << std::endl;
    }
    return cloud;
}

void write_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::io::savePCDFileASCII(filename, *cloud);
    // std::cerr << "Saved " << cloud->points.size() << " data points to " + filename << std::endl<<endl;
}
void write_point_cloud(string filename, PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::io::savePCDFileASCII(filename, *cloud);
    // std::cerr << "Saved " << cloud->points.size() << " data points to " + filename << std::endl<<endl;
}


} // namespace pcl_funcs