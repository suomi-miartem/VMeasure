#include "QPCLVisualizer.h"

QPCLVisualizer::QPCLVisualizer(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	cloud.reset(new PointCloudT);
	passthroughCloud.reset(new PointCloudT);
	euclidCLoud.reset(new PointCloudT);
	normals.reset(new NormalCloudT);

	QObject::connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(Visualize()));
	//sQObject::connect(ui.testButton, &QPushButton::clicked, this, &QPCLVisualizer::euclidCluster);

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
	if (reader.read("test_pcd.pcd", *cloud) == -1) return;

	QMessageBox::information(this, "Information", "Pointcloud was succesfully loaded!");
#pragma endregion

#pragma region Visualization
	//---------------------------------------------------------//
	//----------------------Visualization----------------------//
	//---------------------------------------------------------//
	
	//Инициализация визуализатора
	viewer = InitVisualizer();

	passThrough(cloud, passthroughCloud, float(ui.sbMin->value()), float(ui.sbMax->value()));

	//Евклидова кластеризация с извлечением наибольшего кластера
	euclidCluster(passthroughCloud, euclidCLoud);

	//Вычисление нормалей
	normalEstimation(euclidCLoud, normals);

	getHull(euclidCLoud, normals);

	viewer->addPolygonMesh(mesh, "polygon");
	
	if (!viewer->wasStopped())
		viewer->spinOnce();
#pragma endregion
}
void QPCLVisualizer::colourSliderReleased()
{
	red = ui.hSliderR->value();
	green = ui.hSliderG->value();
	blue = ui.hSliderB->value();

	if (passthroughCloud->empty()) return;

	/*for (auto& point : *passthroughCloud)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}*/

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

//SUPPORT FUNCTIONS
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
	//Фильтрация точек в неотрицательном диапазоне
	pcl::PassThrough<PointT> pass = new pcl::PassThrough<PointT>;
	pass.setInputCloud(src);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.001f, max);
	pass.setNegative(false);
	pass.filter(*dst);

	/*for (auto& point : *dst)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}*/
}

/// <summary>
/// Простая кластеризация по евклидову расстоянию между точками облака.
/// </summary>
void QPCLVisualizer::euclidCluster(PointCloudT::Ptr src, PointCloudT::Ptr dst)
{
	if (src->empty()) return;

	//Настройка параметров кластеризатора и его запуск
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(src);

	std::vector<pcl::PointIndices> ecIndices;
	pcl::EuclideanClusterExtraction<PointT> ec;

	ec.setInputCloud(src);
	ec.setClusterTolerance(0.01); // 1cm
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.extract(ecIndices);

	//Фильтрация наибольшего из отсеянных кластеров
	pcl::PointIndices::Ptr max(new pcl::PointIndices());

	for (std::vector<pcl::PointIndices>::iterator it = ecIndices.begin(); it != ecIndices.end(); ++it) {
		if (it->indices.size() > max->indices.size())
			*max = *it;
	}
		
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(src);
	extract.setIndices(max);
	extract.setNegative(false);
	extract.filter(*dst);
}

void QPCLVisualizer::normalEstimation(PointCloudT::Ptr src, NormalCloudT::Ptr dst)
{
	NormalEstimationT ne;
	ne.setInputCloud(src);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);

	//ne.setRadiusSearch(0.03);

	ne.setKSearch(8);

	ne.compute(*dst);
}

void QPCLVisualizer::getHull(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*src, *nSrc, *cloud_with_normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	gp3.setSearchRadius(0.03);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree);

	gp3.initCompute();
	gp3.reconstruct(mesh);
	/*pcl::ConvexHull<PointT> chull;

	chull.setDimension(3);
	chull.setInputCloud(euclidCLoud);
	chull.setSearchMethod(tree);*/
}