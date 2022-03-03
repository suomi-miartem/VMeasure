#pragma once
#include <thread>

#include <QtConcurrent/QtConcurrent>
#include <QtCore/QFuture>
#include <QtCore/qmutex.h>
//#include <QtCore/QRandomGenerator>
#include <QtWidgets/qlabel.h>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>
#include "ui_QPCLVisualizer.h"

// Point Cloud Library
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class QPCLVisualizer : public QMainWindow
{
    Q_OBJECT

public:
    QPCLVisualizer(QWidget *parent = Q_NULLPTR);
    
public Q_SLOTS:
    void Visualize();
    void colourSliderReleased();
    void passthroughValueChanged();
    void euclidCluster();
    void redSliderValueChanged(int value);
    void greenSliderValueChanged(int value);
    void blueSliderValueChanged(int value);
    //void updateLabel();

private:
    Ui::QPCLVisualizerClass ui;

    pcl::visualization::PCLVisualizer::Ptr InitVisualizer();
    
    void passThrough(PointCloudT::Ptr src, PointCloudT::Ptr dst, float min, float max);

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr passthroughCloud;

    unsigned int red = 255;
    unsigned int green = 255;
    unsigned int blue = 255;
};