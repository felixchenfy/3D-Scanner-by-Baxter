
#include "my_pcl/pcl_commons.h"

#include "my_basics/eigen_funcs.h"
using namespace my_basics;

namespace my_pcl
{
// -- Print
void printCloudSize(PointCloud<PointXYZRGB>::Ptr cloud)
{
    cout << "Cloud size: " << cloud->width << "x" << cloud->height << endl;
}
void printCloudSize(PointCloud<PointXYZ>::Ptr cloud)
{
    cout << "Cloud size: " << cloud->width << "x" << cloud->height << endl;
}

// -- Set point color and pos

void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
}
void setCloudColor(PointCloud<PointXYZRGB>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b)
{
    for (auto &p : cloud->points)
        setPointColor(p, r, g, b);
}
void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}
void setPointPos(pcl::PointXYZ &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(pcl::PointXYZ &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(pcl::PointXYZ &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}

} // namespace my_pcl