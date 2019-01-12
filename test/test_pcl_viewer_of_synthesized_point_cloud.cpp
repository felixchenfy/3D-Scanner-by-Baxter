#include <iostream>
#include <string>

#include <iostream>
#include <boost/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues


// #include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

using namespace std;
using namespace cv;
using namespace Eigen;


string cloud_name="sample cloud";
string str_frame_reference="reference";


// -------- Functions copied to my eigen_trans.h --------
Eigen::Affine3d transCVMatRt2Affine3d(const Mat &R0, const Mat &t)
{
    Mat R=R0.clone();
    Eigen::Affine3d T;
    if(R.rows==3 && R.cols==1)cv::Rodrigues(R, R);
    assert(R.rows==3 && R.cols==3);
    T.linear() = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(reinterpret_cast<const double *>(R.data));
    T.translation() = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
    return T;
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
    // viewer->addCoordinateSystem (1.0);

    Eigen::Affine3f t;
    int viewport=0;
    viewer->addCoordinateSystem(5, t, str_frame_reference, viewport);

    viewer->initCameraParameters();
    return (viewer);
}

int main()
{
    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(basic_cloud_ptr);

    // set up affine matrix
    Eigen::Affine3d T_affine;
    cv::Mat R_vec=(cv::Mat_<double>(3,1)<<0,0,0);
    cv::Mat t=(cv::Mat_<double>(3,1)<<0,0,0);

    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    std::cout << "Generating example point clouds.\n\n";
    uint8_t r(255), g(15), b(15);
    float z = -1.0;
    int cnt=0;
    while( z <= 1.0)
    {
        z += 0.05;
        if(z>1.0){
            z=-1.0;
            basic_cloud_ptr->clear();
            point_cloud_ptr->clear();
        }
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
            basic_point.y = sinf(pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float *>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
        if(viewer->wasStopped())break;

        // change position of the coordinate system
        const double period=10.0;
        t.at<double>(0,0)=sin(cnt/period);
        t.at<double>(1,0)=cos(cnt/period);
        t.at<double>(2,0)-=0.01;
        R_vec.at<double>(0,0)+=0.01;
        T_affine = transCVMatRt2Affine3d(R_vec, t);
        cout << T_affine.matrix() << endl;

        // cv::Mat R;
        // cv::Rodrigues(R_vec, R);
        // TTT.linear() = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(reinterpret_cast<const double *>(R.data));
        // TTT.translation() = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
        // Eigen::Affine3f TTT = T_affine.cast<double>();
        // Eigen::Affine3f TTT(Eigen::Affine3f::Identity());
        Eigen::Affine3f TTT;
        TTT=T_affine.cast<float>();

        viewer->removeCoordinateSystem(str_frame_reference);
        viewer->addCoordinateSystem(5, TTT, str_frame_reference, 0);

        // viewer->spin();
        viewer->removePointCloud(cloud_name);
        viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr, cloud_name);

        viewer->spinOnce(100);
        cout << cnt++ << "th loop, z = " << z << endl;
    }    
    return (1);
}