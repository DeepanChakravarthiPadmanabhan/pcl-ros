#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include<iostream>
#include<thread>

#include <pcl/console/parse.h>

#include <chrono>
int main (int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;

reader.read ("table_scene_mug_stereo_textured_hull.pcd", *cloudSegmented);
// Read the point cloud

// Compute principal directions
Eigen::Vector4f pcaCentroid; // To compute the centroid
pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
Eigen::Matrix3f covariance;
computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

// Transform the original cloud to the origin where the principal components correspond to the axes.
Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
// Get the minimum and maximum points of the transformed cloud.
pcl::PointXYZ minPoint, maxPoint;
pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

// Final transform
const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

// This viewer has 4 windows, but is only showing images in one of them as written here.
pcl::visualization::PCLVisualizer *visu;
visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
//visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
 std::cerr << "X "<<maxPoint.x<<"\n";
 std::cerr << "X "<<minPoint.x<<"\n";
 std::cerr << "Y "<<maxPoint.y<<"\n";
 std::cerr << "Y "<<minPoint.y<<"\n";
 std::cerr << "Z "<<maxPoint.z<<"\n";
 std::cerr << "Z "<<minPoint.z<<"\n";
 while (!visu->wasStopped ())
  {
    visu->spinOnce (100);

  }
  return (0);
}

// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html