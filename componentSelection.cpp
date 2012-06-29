#include "componentSelection.h"


void
segmentComponent(
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

    /*
i indice che scorre i cluster trovati
j indice che scorre i punti dentro il cluster i-esimo

se il punto j-esimo e' quello selezionato in input, il cluster selezionato e' quello i-esimo
*/
    int selectedComponentCluster;
    for (int i=0;i<cluster_indices_out.size();i++)
        for (int j=0;j<cluster_indices_out[i].indices.size();i++)
            if(cluster_indices_out[i].indices.at(j)==selectedPointIndex)
                selectedComponentCluster=i;


    pcl::PointIndices componente=cluster_indices_out[selectedComponentCluster];
    output =  &componente;
}



void
segmentColor (
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
        if(
                abs((*input)[i].r-r)<threshold &&
                abs((*input)[i].g-g)<threshold &&
                abs((*input)[i].b-b)<threshold
                )
        output->indices.push_back(i);
    }

}
