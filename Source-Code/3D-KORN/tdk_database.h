#ifndef TDK_DATABASE_H
#define TDK_DATABASE_H

#include <QObject>
#include <QDebug>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include "tdk_edit.h"

class TDK_Database : public QObject
{
    Q_OBJECT
public:

    explicit TDK_Database(QObject *parent = 0);
    ~TDK_Database();

    static void     mf_StaticAddPointCloud              (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr,
                                                        QString pointCloudName = "U1425_AUTOGENERATE");

    static void     mf_StaticAddRegisteredPointCloud    (pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredPointCloudPtr,
                                                        QString registeredPointCloudName = "U1425_AUTOGENERATE");

    static void     mf_StaticAddMesh                    (pcl::PolygonMesh::Ptr meshPtr,
                                                        QString meshName = "U1425_AUTOGENERATE");

    //TO BE IMPLEMENTED
    static std::vector<TDK_Edit*>                               mv_EditHistoryVector;

    static unsigned int                                         mv_NumberOfAutogeneratedPointClouds;
    static std::vector<QString>                                 mv_PointCloudsName;
    static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  mv_PointCloudsVector;

    static unsigned int                                         mv_NumberOfAutogeneratedRegisteredPointClouds;
    static std::vector<QString>                                 mv_RegisteredPointCloudsName;
    static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  mv_RegisteredPointCloudsVector;

    static unsigned int                                         mv_NumberOfAutogeneratedMeshes;
    static std::vector<QString>                                 mv_MeshesName;
    static std::vector<pcl::PolygonMesh::Ptr>                   mv_MeshesVector;


};

#endif // TDK_DATABASE_H
