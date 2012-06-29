#include "componentSelection.h"

void
segmentComponent
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    double threshold
)
{

    vector<pcl::PointIndices> cluster_indices_out;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (threshold);
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (input->size());
    ec.setInputCloud (input);
    ec.extract (cluster_indices_out);


//    i cluster index
//    j index of the point in the i-th cluster
//    if the j-th point is the one selected the cluster selected is the i-th

    int selectedClusterIndex;
    for (int i=0;i<cluster_indices_out.size();i++)
        for (int j=0;j<cluster_indices_out[i].indices.size();j++)
            if(cluster_indices_out[i].indices.at(j)==selectedPointIndex)
            {
                selectedClusterIndex=i;
                break;
            }
//    TO DO: aggiungere un break per il ciclo più esterno, o un controllo con un flag booleano se ha finito nei cicli precedenti

    pcl::PointIndicesPtr temp(&cluster_indices_out[selectedClusterIndex]);
    output = temp;
//    *output=cluster_indices_out[selectedClusterIndex];
}



void
segmentColor
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    int threshold
)

{
    pcl::PointXYZRGB selectedPoint = (*input)[selectedPointIndex];
    int r = selectedPoint.r;
    int g = selectedPoint.g;
    int b = selectedPoint.b;

    for (int i = 0; i < input->size(); i++){
        if
        (
                abs((*input)[i].r-r)<threshold &&
                abs((*input)[i].g-g)<threshold &&
                abs((*input)[i].b-b)<threshold
        )
        output->indices.push_back(i);
    }

}
