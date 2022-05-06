#include "QPCLVisualizer.h"

QPCLVisualizer::QPCLVisualizer(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	QObject::connect(ui.pushButton1, SIGNAL(clicked()), this, SLOT(processEmpty()));
	QObject::connect(ui.pushButton2, SIGNAL(clicked()), this, SLOT(processFull()));
	QObject::connect(ui.testButton, SIGNAL(clicked()), this, SLOT(volumeEstimation()));
}

//SLOTS
void QPCLVisualizer::processEmpty() 
{
	emptyCloud.reset(new PointCloudT);
	bottomPlane.reset(new PointCloudT);
	bottomPlaneContour.reset(new PointCloudT);
	topPlane.reset(new PointCloudT);
	topPlaneOuterContour.reset(new PointCloudT);

	//Загрузка облака
	pcl::PCDReader reader;
	if (reader.read("Empty.pcd", *emptyCloud) == -1) return;

	auto pass = passThrough(emptyCloud, float(ui.sbMin->value()), float(ui.sbMax->value()));

	//Вычисление нормалей
	//normals = normalEstimation(pass);

	auto planes = euclidCluster(pass, 0.003);
	topPlane = planes[1];
	bottomPlane = planes[0];

	///*for (int i = 0; i < planes.size(); i++)
	//{
	//	float z = 0;

	//	for (int j = 0; j < planes[i]->size(); j++)
	//	{
	//		z += (*planes[i])[j].z;
	//	}

	//	z /= (float)planes[i]->size();

	//	for (int j = 0; j < planes[i]->size(); j++)
	//	{
	//		(*planes[i])[j].z = z;
	//	}
	//}*/

	////Получение отдельных контуров
	bottomPlaneContour = getBoundary(bottomPlane, normalEstimation(bottomPlane)); //Нижний контур
	auto topPlaneContours = euclidCluster(getBoundary(topPlane, normalEstimation(topPlane)), 0.005); //Верхние контуры
	topPlaneOuterContour = topPlaneContours[0]->size() > topPlaneContours[1]->size() ? topPlaneContours[0] : topPlaneContours[1]; //Внешний

	//pcl::visualization::PCLVisualizer::Ptr vis = InitVisualizer();
	
	//vis->addPointCloud(topPlane);
	//vis->addPointCloudNormals<PointT, pcl::Normal>(pass, normals, 10, 0.05, "normals");

	/*while (!vis->wasStopped())
	{
		vis->spinOnce(100);
		QThread::currentThread()->msleep(100);
	}*/
}
void QPCLVisualizer::processFull()
{
	fullCloud.reset(new PointCloudT);
	fullTopPlaneOuterContour.reset(new PointCloudT);

	//Загрузка облака
	pcl::PCDReader reader;
	if (reader.read("Full1.pcd", *fullCloud) == -1) return;

	fullTopPlane = passThrough(fullCloud, float(ui.sbMin->value()), float(ui.sbMax->value()));

	auto boundaries = euclidCluster(getBoundary(fullTopPlane, normalEstimation(fullTopPlane)), 0.005);

	fullTopPlaneOuterContour = boundaries[0];
	/*pcl::visualization::PCLVisualizer::Ptr vis = InitVisualizer();
	vis->addPointCloud(pass);

	while (!vis->wasStopped())
	{
		vis->spinOnce(100);
		QThread::currentThread()->msleep(100);
	}*/
}
void QPCLVisualizer::volumeEstimation()
{
	//PC align
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(topPlaneOuterContour);
	icp.setInputTarget(fullTopPlaneOuterContour);
	icp.setMaximumIterations(200);
	PointCloudT::Ptr aligned(new PointCloudT);
	icp.align(*aligned);

	//set transform to the target boundaries and planes
	auto transform = icp.getFinalTransformation();

	pcl::transformPointCloud(*bottomPlane, *bottomPlane, transform);

	pcl::transformPointCloud(*bottomPlaneContour, *bottomPlaneContour, transform);

	//Merging all clouds needed
	auto wall = makeWall(aligned, bottomPlaneContour, bStep(bottomPlaneContour));

	PointCloudT::Ptr unitedCloud(new PointCloudT);
	PointCloudT::Ptr result(new PointCloudT);

	pcl::concatenate(*fullTopPlane, *bottomPlane, *unitedCloud);
	pcl::concatenate(*unitedCloud, *wall, *unitedCloud);

	//Search convex hull and compute volume
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	//pcl::ConvexHull<PointT> chull;
	pcl::ConcaveHull<PointT> chull;
	chull.setDimension(3);
	chull.setInputCloud(unitedCloud);
	chull.setSearchMethod(tree);
	//chull.setComputeAreaVolume(true);
	chull.setAlpha(0.01);
	chull.reconstruct(mesh);

	//std::stringstream ss;
	//ss << "Point cloud volume is: " << chull.getTotalVolume() << " m3";
	//QMessageBox::information(this, "INFO", QString::fromStdString((ss.str())));

	auto vis = InitVisualizer();
	
	//vis->addPointCloud(result);
	vis->addPolygonMesh(mesh);
		
	while (!vis->wasStopped())
	{
		vis->spinOnce(100);
		QThread::currentThread()->msleep(100);
	}
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
	//viewer->addCoordinateSystem(0.5);
	viewer->initCameraParameters();
	return (viewer);
}

/// <summary>
/// Пороговый фильтр по вектору Z.
/// </summary>
PointCloudT::Ptr QPCLVisualizer::passThrough(PointCloudT::Ptr src, float min, float max)
{
	PointCloudT::Ptr result(new PointCloudT);

	//Фильтрация точек в неотрицательном диапазоне
	pcl::PassThrough<PointT> pass = new pcl::PassThrough<PointT>;
	pass.setInputCloud(src);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.01f, max);
	pass.setNegative(false);
	pass.filter(*result);
	/*for (auto& point : *dst)
	{
		point.r = red;
		point.g = green;
		point.b = blue;
	}*/

	return result;
}

/// <summary>
/// Простая кластеризация по евклидову расстоянию между точками облака.
/// </summary>
std::vector<PointCloudT::Ptr, Eigen::aligned_allocator<PointCloudT::Ptr>> QPCLVisualizer::euclidCluster(PointCloudT::Ptr src, double tolerance)
{
	std::vector<PointCloudT::Ptr, Eigen::aligned_allocator<PointCloudT::Ptr>> result;

	//Настройка параметров кластеризатора и его запуск
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(src);

	std::vector<pcl::PointIndices> indices;
	pcl::EuclideanClusterExtraction<PointT> ec;

	ec.setInputCloud(src);
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(100);
	//ec.setMaxClusterSize((int)((src->size() / 100) * 70));
	ec.setSearchMethod(tree);
	ec.extract(indices);
		
	for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it)
	{
		PointCloudT::Ptr subCloud(new PointCloudT);

		for (auto idx : it->indices)
			subCloud->push_back((*src)[idx]);

		result.push_back(subCloud);
	}
	
	return result;
}

void QPCLVisualizer::rgSeg(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc)
{
	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> uni(50, 255);

	int r, g, b;

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(8);
	reg.setInputCloud(src);
	//reg.setIndices (indices);
	reg.setInputNormals(nSrc);
	reg.setSmoothnessThreshold(5.5 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	std::vector <pcl::PointIndices> outClusters;
	reg.extract(clusters);

	for (std::vector<pcl::PointIndices>::iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		r = uni(rng);
		g = uni(rng);
		b = uni(rng);
		/*for (std::vector<int>::const_iterator idx = it->indices.begin(); idx != it->indices.end(); ++idx)
		{
			(*src)[*idx].r = r;
			(*src)[*idx].g = g;
			(*src)[*idx].b = b;
		}*/
	}
}

NormalCloudT::Ptr QPCLVisualizer::normalEstimation(PointCloudT::Ptr src)
{
	NormalEstimationT ne;
	NormalCloudT::Ptr result(new NormalCloudT);
	ne.setInputCloud(src);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.005);
	
	//ne.setKSearch(8);

	ne.compute(*result);

	return result;
}

void QPCLVisualizer::getHull(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*src, *nSrc, *cloud_with_normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr pointNormalTree(new pcl::search::KdTree<pcl::PointNormal>);
	pcl::search::KdTree<PointT>::Ptr pointTree(new pcl::search::KdTree<PointT>);
	pointNormalTree->setInputCloud(cloud_with_normals);
	pointTree->setInputCloud(src);
	
	/*pcl::OrganizedFastMesh<PointT> ofm;
	ofm.setInputCloud(src);
	ofm.setMaxEdgeLength(1.5);
	ofm.setTrianglePixelSize(2);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointT>::TRIANGLE_ADAPTIVE_CUT);
	ofm.reconstruct(mesh);*/

	//pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	//gp3.setSearchRadius(0.8);
	//gp3.setMu(2.5);
	//gp3.setMaximumNearestNeighbors(100);
	//gp3.setMaximumSurfaceAngle(M_PI / 2); // 90 degrees
	//gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	//gp3.setMaximumAngle(7 * M_PI / 6); // 210 degrees
	//gp3.setNormalConsistency(false);
	//gp3.setInputCloud(cloud_with_normals);
	//gp3.setSearchMethod(tree);
	//gp3.initCompute();
	//gp3.reconstruct(mesh);

	/*pcl::GridProjection<pcl::PointXYZRGBNormal> gp;
	gp.setInputCloud(cloud_with_normals);
	gp.setPaddingSize(3);
	gp.setResolution(0.003);
	gp.setSearchMethod(tree);
	gp.setNearestNeighborNum(50);
	gp.setMaxBinarySearchLevel(10);
	gp.initCompute();
	gp.reconstruct(mesh);*/

	pcl::ConvexHull<PointT> chull;
	chull.setDimension(3);
	chull.setInputCloud(src);
	chull.setSearchMethod(pointTree);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(mesh);
}

PointCloudT::Ptr QPCLVisualizer::getBoundary(PointCloudT::Ptr src, NormalCloudT::Ptr nSrc)
{
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

	pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> bEstimation;
	bEstimation.setInputCloud(src);
	bEstimation.setInputNormals(nSrc);
	bEstimation.setRadiusSearch(0.01);
	bEstimation.setSearchMethod(tree);
	bEstimation.compute(boundaries);

	PointCloudT::Ptr output(new PointCloudT);

	for (size_t i = 0; i < src->points.size(); ++i)
	{
		if (boundaries[i].boundary_point > 0)
		{
			output->push_back(src->points[i]);
		}
	}

	return output;
}

float QPCLVisualizer::bStep(PointCloudT::Ptr src)
{
	float result = 0;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(src);
	pcl::Indices indices;
	std::vector<float> distances;
	
	for (int i = 0; i < src->size(); i++)
		if (tree->nearestKSearch(i, 3, indices, distances) == 3)
			for (auto it = distances.begin() + 1; it != distances.end(); ++it)
				result += sqrt(*it);

	return (result / (src->size() * 2));
}

PointCloudT::Ptr QPCLVisualizer::sort(PointCloudT::Ptr src)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(src);

	PointCloudT::Ptr result(new PointCloudT);

	return result;
}

PointCloudT::Ptr QPCLVisualizer::makeWall(PointCloudT::Ptr src1, PointCloudT::Ptr src2, float step)
{
	PointCloudT::Ptr result(new PointCloudT);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(src2);
	pcl::Indices indices;
	std::vector<float> distances;

	for (auto& pointA : *src1)
	{
		tree->nearestKSearch(pointA, 10, indices, distances); 
		
		for (auto idx : indices)
		{
			auto pointB = (*src2)[idx];//ближайшая в соседнем контуре точка

			int steps = static_cast<int>(sqrt(distances[0]) / step);

			for (int i = 1; i < steps; i++)
			{
				float x = pointA.x + i * ((pointB.x - pointA.x) / steps);
				float y = pointA.y + i * ((pointB.y - pointA.y) / steps);
				float z = pointA.z + i * ((pointB.z - pointA.z) / steps);

				result->push_back(PointT(x, y, z));
			}
		}
	}

	return result;
}

PointCloudT::Ptr QPCLVisualizer::getFrame()
{
	//device configuration
	rs2::context ctx;
	auto devices = ctx.query_devices();
	auto dev = devices[0];
	auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	size_t device_count = devices.size();
	if (!device_count)
	{
		QMessageBox::information(this, "Attention!", "There's no plugged devices!");

		return nullptr;
	}

	//advanced device configuration/load json file
	auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
	if (!advanced_mode_dev.is_enabled())
	{
		// If not, enable advanced-mode
		advanced_mode_dev.toggle_advanced_mode(true);
		std::cout << "Advanced mode enabled. " << std::endl;
	}
	std::ifstream t("preset.json");
	std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	advanced_mode_dev.load_json(preset_json);

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_device(serial);

	//pipeline configuration and running
	rs2::pipeline pipe;
	auto profile = pipe.start(cfg);

	//get filtered depth frame
	rs2::pointcloud pc;
	rs2::points points;

	//auto decimation = rs2::decimation_filter(2.0F);
	auto spatial_filter = rs2::spatial_filter(0.25F, 20.0F, 5.0F, 0.0F);
	auto temporal_filter = rs2::temporal_filter(0.32F, 20.0F, 4);
	auto hole_filter = rs2::hole_filling_filter(2);
	auto depth_to_disparity = rs2::disparity_transform(true);
	auto disparity_to_depth = rs2::disparity_transform(false);

	rs2::frameset frameset;
	rs2::frame filtered;

	for (int i = 0; i < 120; i++)
	{
		frameset = pipe.wait_for_frames();
		auto depth = frameset.get_depth_frame();
		filtered = depth
			.apply_filter(depth_to_disparity)
			.apply_filter(spatial_filter)
			.apply_filter(temporal_filter)
			.apply_filter(disparity_to_depth);
		//filtered = hole_filter.process(filtered);
	}

	pipe.stop();

	// Generate the pointcloud and texture mappings
	points = pc.calculate(filtered);
	auto result = points_to_pcl(points);
	return result;
}

PointCloudT::Ptr QPCLVisualizer::points_to_pcl(const rs2::points& points)
{
	PointCloudT::Ptr cloud(new PointCloudT);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}