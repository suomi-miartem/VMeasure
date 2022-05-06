#pragma once
#include <thread>
#include <math.h>

#include<librealsense2/rs.hpp>
#include<librealsense2/rs_advanced_mode.hpp>

#include <QtConcurrent/QtConcurrent>
#include <QtCore/QFuture>
#include <QtCore/qmutex.h>
#include <QtWidgets/qlabel.h>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>
#include "ui_QPCLVisualizer.h"

// Point Cloud Library
#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::NormalEstimation<PointT, pcl::Normal> NormalEstimationT;
typedef pcl::PointCloud<pcl::Normal> NormalCloudT;

class QPCLVisualizer : public QMainWindow
{
    Q_OBJECT

public:
    QPCLVisualizer(QWidget *parent = Q_NULLPTR);
    
public Q_SLOTS:
    void processEmpty();
    void processFull();
    void volumeEstimation();

private:
    Ui::QPCLVisualizerClass ui;

    pcl::visualization::PCLVisualizer::Ptr InitVisualizer();
    
    PointCloudT::Ptr passThrough(PointCloudT::Ptr src, float min, float max);

    std::vector<PointCloudT::Ptr, Eigen::aligned_allocator<PointCloudT::Ptr>> euclidCluster(PointCloudT::Ptr src, double tolerance);
    
    NormalCloudT::Ptr normalEstimation(PointCloudT::Ptr src);

    void getHull(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc);

    void rgSeg(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc);

    PointCloudT::Ptr restoreWalls(PointCloudT::Ptr contour1, PointCloudT::Ptr contour2);

    PointCloudT::Ptr sort(PointCloudT::Ptr src);

    PointCloudT::Ptr getBoundary(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc);

    PointCloudT::Ptr makeWall(PointCloudT::Ptr src1, PointCloudT::Ptr src2, float step);

    PointCloudT::Ptr getFrame();

    PointCloudT::Ptr points_to_pcl(const rs2::points& points);

    float bStep(PointCloudT::Ptr src);

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr emptyCloud;
    PointCloudT::Ptr fullCloud;
    PointCloudT::Ptr bottomPlane;
    PointCloudT::Ptr bottomPlaneContour;
    PointCloudT::Ptr topPlane;
    PointCloudT::Ptr topPlaneOuterContour;
    PointCloudT::Ptr fullTopPlane;
    PointCloudT::Ptr fullTopPlaneOuterContour;
    //PointCloudT::Ptr 
    pcl::PolygonMesh mesh;

    unsigned int red = 255;
    unsigned int green = 0;
    unsigned int blue = 0;
};