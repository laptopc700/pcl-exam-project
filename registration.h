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

/*
void
changeColor (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
             int r,
             int g,
             int b);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                                                   double leafSize,
                                                   int verbosity
                                                   );

void segmentation ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented,
                   int verbosity
                   );

void detectKeypoints ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
                       int verbosity
                       );

void extractDescriptors ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr features,
                          int verbosity
                          );

void findCorrespondences ( pcl::PointCloud<pcl::FPFHSignature33>::Ptr source,
                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target,
                           vector<int>& correspondences,
                           int verbosity
                           );

void filterCorrespondences ( pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
                             vector<int>& source2target_ , vector<int>& target2source_ ,
                             pcl::CorrespondencesPtr correspondences_,
                             int verbosity
                             );

Eigen::Matrix4f determineInitialTransformation ( pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_ ,
                                                 pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
                                                 pcl::CorrespondencesPtr correspondences_,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_segmented_ ,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_ ,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_,
                                                 int verbosity
                                                );

Eigen::Matrix4f determineFinalTransformation ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_ ,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_registered_,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_,
                                               int verbosity
                                               );

//#################################################################################################
//##############################################################   R E G I S T E R   ##############
//#################################################################################################
*/
int
registerSourceToTarget ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr source ,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered,
                         int verbosity,
                         int compute_target
                         );

#endif // REGISTRATION_H
