#include "componentMatch.h"
#include "componentSelection.h"
#include <pcl/common/geometry.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

void
componentMatch
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
    pcl::PointIndices::Ptr inputComponentIndices,
    int clickedIndex,
    double cluThreshold,
    int colThreshold,
    pcl::PointIndices::Ptr outputComponentIndices
)
{

//pcl::KdTreeFLANN<pcl::PointXYZRGB> treeflann=new pcl::KdTreeFLANN<pcl::PointXYZRGB>();
//pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

pcl::PointXYZRGB clickedPoint = targetCloud->at(clickedIndex);
int originalSize = registeredSource->size();
registeredSource->push_back(clickedPoint); //aggiungo il punto cliccato, usandolo per la segmentazione



//pcl::PointIndices::Ptr k_indices(new pcl::PointIndices);
//vector<float> k_distances;

//NON VA IL KDTREE, NON VA L'OCTREE NON VA UN CAZZO!
//treeflann.setEpsilon(0.1);
//treeflann.setInputCloud(registeredSource);
//treeflann.nearestKSearch(registeredSource->at(originalSize),50,k_indices->indices,k_distances);
////tree->nearestKSearch(registeredSource,originalSize,50,k_indices->indices,k_distances);
////cerca il punto della source piu' vicino (dist&color) a quello cliccato nella target
//for (int i=0; i<k_indices->indices.size();i++)
//    if (colorOk(clickedPoint,registeredSource->at(k_indices->indices.at(i))))
//    {
//        cout << "Trovato punto alla distanza: "<<k_distances.at(i)<<endl;
//        //break;
//    }
//    else
//    {
//        cout << "No! ";
//    }


//component selection
pcl::PointIndices::Ptr cluIndices(new pcl::PointIndices);
cout<<cluThreshold;
segmentCluster(registeredSource,cluIndices,originalSize,cluThreshold);

pcl::PointIndices::Ptr colIndices(new pcl::PointIndices);
segmentColor(registeredSource,colIndices,originalSize,colThreshold);

intersectIndices(cluIndices,colIndices,outputComponentIndices);

cout << "Size component on target cloud:"<<inputComponentIndices->indices.size()<<endl;
cout << "Size component on source cloud:"<<outputComponentIndices->indices.size()<<endl;
}

//returns true if the color of the two points are similar (+-10 RGB)
//bool colorOk(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2){
//    int threshold=10;
//    if
//        (
//         abs(p1.r-p2.r)<threshold &&
//         abs(p1.g-p2.g)<threshold &&
//         abs(p1.b-p2.b)<threshold
//        )
//        return true;
//    else
//        return false;
//}
