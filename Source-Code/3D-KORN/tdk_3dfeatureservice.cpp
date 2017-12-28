#include "tdk_3dfeatureservice.h"

TDK_3DFeatureService::TDK_3DFeatureService()
{

}

TDK_3DFeatureService::~TDK_3DFeatureService()
{

}

// Functions for setting point cloud that we will use for feature detection
void TDK_3DFeatureService::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloudPtr)
{
    mv_InPointCloudPtr = inPointCloudPtr;
}

void TDK_3DFeatureService::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &inPointCloud)
{
    mv_InPointCloudPtr= boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(inPointCloud);
}

void TDK_3DFeatureService::setRadiusOfGradientSearch(float radius)
{
    mv_RadiusOfGradientSearch = radius;
}

void TDK_3DFeatureService::setRadiusOfNormalSearch(float radius)
{
    mv_RadiusOfNormalSearch = radius;
}

void TDK_3DFeatureService::setRadiusOfRIFTSearch(float radius)
{
    mv_RadiusOfRIFTSearch = radius;
}

float TDK_3DFeatureService::getRadiusOfGradientSearch()
{
    return mv_RadiusOfGradientSearch;
}

float TDK_3DFeatureService::getRadiusOfNormalSearch()
{
    return mv_RadiusOfNormalSearch;
}

float TDK_3DFeatureService::getRadiusOfRIFTSearch()
{
    return mv_RadiusOfRIFTSearch;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr& TDK_3DFeatureService::getFeatures()
{
    if (!!mv_InPointCloudPtr)
    {
        auto features = runRIFTFeatureDetection();
        return features;
    }
    else
    {
        throw std::runtime_error("Value of mv_InPointCloudPtr should not be an empty pointer");
    }
}


pcl::PointCloud<pcl::PointXYZI>::Ptr&
TDK_3DFeatureService::runRIFTFeatureDetection()
{
    pcl::PointCloud<pcl::PointXYZI> pCloudXYZI{};

    for_each(mv_InPointCloudPtr->begin(),
            mv_InPointCloudPtr->end(),
            [&pCloudXYZI] (pcl::PointXYZRGB pRGB) {
        pcl::PointXYZI pI{};
        pcl::PointXYZRGBtoXYZI(pRGB, pI);
        pCloudXYZI.push_back(pI);
    });
    pCloudXYZI.is_dense = mv_InPointCloudPtr->is_dense;
    pCloudXYZI.sensor_origin_ = mv_InPointCloudPtr->sensor_origin_;
    pCloudXYZI.sensor_orientation_ = mv_InPointCloudPtr->sensor_orientation_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(&pCloudXYZI);

    // Estimate the surface normals
     pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
     norm_est.setInputCloud(cloud);
     pcl::search::KdTree<pcl::PointXYZI>::Ptr treept1 (new pcl::search::KdTree<pcl::PointXYZI> (false));
     norm_est.setSearchMethod(treept1);
     norm_est.setRadiusSearch(mv_RadiusOfNormalSearch);
     norm_est.compute(*cloud_n);

     qDebug() <<" Surface normals estimated";
     qDebug() <<" with size "<< cloud_n->points.size();

     // Estimate the Intensity Gradient
     pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
     pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
     gradient_est.setInputCloud(cloud);
     gradient_est.setInputNormals(cloud_n);
     pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
     gradient_est.setSearchMethod(treept2);
     gradient_est.setRadiusSearch(mv_RadiusOfGradientSearch);
     gradient_est.compute(*cloud_ig);
     qDebug() <<" Intesity Gradient estimated";
     qDebug() <<" with size "<< cloud_ig->points.size();


     // Estimate the RIFT feature
     pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32> > rift_est;
     pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
     rift_est.setSearchMethod(treept3);
     rift_est.setRadiusSearch(mv_RadiusOfRIFTSearch);
     rift_est.setNrDistanceBins (4);
     rift_est.setNrGradientBins (8);
     rift_est.setInputCloud(cloud);
     rift_est.setInputGradient(cloud_ig);
     pcl::PointCloud<pcl::Histogram<32> > rift_output;
     rift_est.compute(rift_output);

     qDebug() <<" RIFT feature estimated";
     qDebug() <<" with size "<<rift_output.points.size();

     // Display and retrieve the rift descriptor vector for the first point
     pcl::Histogram<32> first_descriptor = rift_output.points[0];
     //qDebug() << first_descriptor;
     return cloud;
}
