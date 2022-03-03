#include "QPCLVisualizer.h"

QPCLVisualizer::QPCLVisualizer(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	cloud.reset(new PointCloudT);
	passthroughCloud.reset(new PointCloudT);

	QObject::connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(Visualize()));
	QObject::connect(ui.testButton, &QPushButton::clicked, this, &QPCLVisualizer::euclidCluster);

	QObject::connect(ui.hSliderR, &QSlider::sliderMoved, ui.lcdR, qOverload<int>(&QLCDNumber::display));
	QObject::connect(ui.hSliderR, &QSlider::sliderReleased, this, &QPCLVisualizer::colourSliderReleased);

	QObject::connect(ui.hSliderG, &QSlider::sliderMoved, ui.lcdG, qOverload<int>(&QLCDNumber::display));
	QObject::connect(ui.hSliderG, &QSlider::sliderReleased, this, &QPCLVisualizer::colourSliderReleased);

	QObject::connect(ui.hSliderB, &QSlider::sliderMoved, ui.lcdB, qOverload<int>(&QLCDNumber::display));
	QObject::connect(ui.hSliderB, &QSlider::sliderReleased, this, &QPCLVisualizer::colourSliderReleased);

	QObject::connect(ui.sbMin, &QDoubleSpinBox::valueChanged, this, &QPCLVisualizer::passthroughValueChanged);
	QObject::connect(ui.sbMax, &QDoubleSpinBox::valueChanged, this, &QPCLVisualizer::passthroughValueChanged);
}

//SLOTS
void QPCLVisualizer::Visualize() 
{
	//Вспомогательные штучки
	using namespace std::chrono_literals;
#pragma region PCD load
	pcl::PCDReader reader;
	reader.read("test_pcd.pcd", *cloud);
	if (reader.read("test_pcd.pcd", *cloud) == -1) return;

	QMessageBox::information(this, "Information", "Pointcloud was succesfully loaded!");
#pragma endregion

	//Basic cloud generation
#pragma region Point cloud generation
	//---------------------------------------------------------//
	//------------------Point cloud generation-----------------//
	//---------------------------------------------------------//
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	// We're going to make an ellipse extruded along the z-axis. The colour for
	// the XYZRGB cloud will gradually go from red to green to blue.
	std::uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * std::cos(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
				static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
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
	}
	basic_cloud_ptr->width = basic_cloud_ptr->size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = point_cloud_ptr->size();
	point_cloud_ptr->height = 1;
#pragma endregion

	passThrough(cloud, passthroughCloud, float(ui.sbMin->value()), float(ui.sbMax->value()));

#pragma region Visualization
	//---------------------------------------------------------//
	//----------------------Visualization----------------------//
	//---------------------------------------------------------//
	viewer = InitVisualizer();
	viewer->addPointCloud(passthroughCloud);

	//Добавить облако
	for (auto& point : *passthroughCloud)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}

	
	if(!viewer->addPointCloud<pcl::PointXYZRGB>(passthroughCloud, "cloud"))
		viewer->updatePointCloud(passthroughCloud,"cloud");

	if (!viewer->wasStopped())
		viewer->spinOnce();

	std::stringstream ss;
	ss << "Cloud width: " << cloud->width << ", height: " << cloud->height;
	ui.testLine->setText(QString::fromStdString(ss.str()));
#pragma endregion
}
void QPCLVisualizer::colourSliderReleased()
{
	red = ui.hSliderR->value();
	green = ui.hSliderG->value();
	blue = ui.hSliderB->value();

	if (passthroughCloud->empty()) return;

	for (auto& point : *passthroughCloud)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}

	viewer->updatePointCloud(passthroughCloud, "cloud");
	viewer->spinOnce();
}
void QPCLVisualizer::passthroughValueChanged()
{
	double min = ui.sbMin->value();
	double max = ui.sbMax->value();

	if (passthroughCloud->empty())
		return;

	passThrough(cloud, passthroughCloud, float(min), float(max));

	viewer->updatePointCloud(passthroughCloud, "cloud");
	viewer->spinOnce();
}
void QPCLVisualizer::redSliderValueChanged(int value)
{
	red = value;
}
void QPCLVisualizer::greenSliderValueChanged(int value)
{
	green = value;
}
void QPCLVisualizer::blueSliderValueChanged(int value)
{
	blue = value;
}
//void QPCLVisualizer::updateLabel()
//{
//	ui.nLabel->setText(QString::number(float(ui.hSliderN->value()) / 1000.0));
//}

//SUPPORTING FUNCTIONS

/// <summary>
/// Инициализация визуализатора из простого облака точек.
/// </summary>
pcl::visualization::PCLVisualizer::Ptr QPCLVisualizer::InitVisualizer()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(0.5);
	viewer->initCameraParameters();
	
	return (viewer);
}

/// <summary>
/// Пороговый фильтр по вектору Z.
/// </summary>
void QPCLVisualizer::passThrough(PointCloudT::Ptr src, PointCloudT::Ptr dst, float min, float max)
{
	pcl::PassThrough<PointT> pass = new pcl::PassThrough<PointT>;
	pass.setInputCloud(src);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min, max);
	pass.filter(*dst);

	for (auto& point : *dst)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}
}

/// <summary>
/// Простая кластеризация по евклидову расстоянию между точками облака.
/// </summary>
void QPCLVisualizer::euclidCluster()
{
	if (passthroughCloud->empty()) return;

	std::stringstream ss;

	//настройка параметров кластеризатора и его запуск
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(passthroughCloud);

	std::vector<pcl::PointIndices> ecIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

	ec.setClusterTolerance(0.01); // 1cm
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(passthroughCloud);
	ec.extract(ecIndices);

	//Окраска кластеров
	PointCloudT::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
	QRandomGenerator rnd;
	int idxRed = 0;
	int idxGreen = 0;
	int idxBlue = 0;

	pcl::PointIndices::Ptr max (new pcl::PointIndices ());
	//max->indices = std::max_element(ecIndices.cbegin(), ecIndices.cend())->indices; //pcl::PointIndicesPtr(new pcl::PointIndices);

	for (std::vector<pcl::PointIndices>::const_iterator it = ecIndices.begin(); it != ecIndices.end(); ++it) {
		if (it->indices.size() > max->indices.size())
			max->indices = it->indices;
	}

	for (const auto& idx : max->indices)
		cloud_cluster->push_back((*passthroughCloud)[idx]);

	/*pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(passthroughCloud);
	extract.setIndices(max);
	extract.setNegative(false);
	extract.filter(*cloud_cluster);*/

	//Наибольший кластер
	

	/*for (const auto& idx : max->indices) {
		if (idx > passthroughCloud->size() || idx < 0) {
			ss.clear();
			ss << "Index " << idx << " is not valid. Size of the cloud is " << passthroughCloud->size();
			QMessageBox::information(this, QString("Information"), QString::fromStdString(ss.str()));
		}
	}*/
	
	/*viewer->updatePointCloud(cloud_cluster, "cloud");
	viewer->spinOnce();*/

	
	/*for (std::vector<pcl::PointIndices>::const_iterator it = ecIndices.begin(); it != ecIndices.end(); ++it) {
		idxRed = rnd.bounded(0, 256);
		idxGreen = rnd.bounded(0, 256);
		idxBlue = rnd.bounded(0, 256);
		
		for (const auto& idx : it->indices) {
			(*passthroughCloud)[idx].r = idxRed;
			(*passthroughCloud)[idx].g = idxGreen;
			(*passthroughCloud)[idx].b = idxBlue;
		}

		viewer->updatePointCloud(passthroughCloud, "cloud");
		viewer->spinOnce();
	}*/
}
