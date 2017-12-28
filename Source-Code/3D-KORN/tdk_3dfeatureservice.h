#ifndef TDK_3DFEATURESERVICE_H
#define TDK_3DFEATURESERVICE_H

#include <boost/shared_ptr.hpp>
#include <exception>

#include <pcl/common/io.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rift.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <QDebug>


class TDK_3DFeatureService
{
public:
    TDK_3DFeatureService();
    ~TDK_3DFeatureService();

    //methods for setting an input point cloud before running the feature detection
    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloudPtr);
    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &inPointCloud);

    //setting and getting configuration paramaters
    void setRadiusOfGradientSearch(float radius);
    void setRadiusOfNormalSearch(float radius);
    void setRadiusOfRIFTSearch(float radius);

    float getRadiusOfGradientSearch();
    float getRadiusOfNormalSearch();
    float getRadiusOfRIFTSearch();


    //method for running feature detection
    pcl::PointCloud<pcl::PointXYZI>::Ptr& getFeatures();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mv_InPointCloudPtr;
    float mv_RadiusOfNormalSearch;
    float mv_RadiusOfGradientSearch;
    float mv_RadiusOfRIFTSearch;

    pcl::PointCloud<pcl::PointXYZI>::Ptr& runRIFTFeatureDetection();
};

#endif // TDK_3DFEATURESERVICE_H
