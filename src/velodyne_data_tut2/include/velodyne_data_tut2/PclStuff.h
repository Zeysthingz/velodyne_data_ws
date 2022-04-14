#ifndef PCL_STUFF_H
#define PCL_STUFF_H
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <vector>
#include <tuple>

#include <pcl/point_cloud.h>

//ground removing
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>

//downsampling
#include <pcl/filters/voxel_grid.h>



class PclStuff {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  static Cloud::Ptr
  Downsample(const Cloud::ConstPtr &cloud_in, float leaf_size);

  static Cloud::Ptr
  GroundRemover(Cloud::ConstPtr cloud_in, float treshold = 0.2);

  static Cloud::Ptr
  GroundFinder(Cloud::ConstPtr cloud_in, float treshold = 0.2);

  static Cloud::Ptr
  EucledianCluster(Cloud::ConstPtr cloud_in);
};


#endif //PCL_STUFF_H
