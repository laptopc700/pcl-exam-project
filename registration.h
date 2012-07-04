#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;


int
registerSourceToTarget ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr source ,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered,
                         int verbosity,
                         int compute_target
                         );


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
voxelCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafSize, int verbosity);
#endif // REGISTRATION_H
