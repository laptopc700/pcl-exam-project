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
    cout << "segmentComponent..." << flush;
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
    cout << "OK!"<<endl;
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
    cout << "segmentColor..." << flush;
    pcl::PointXYZRGB selectedPoint = (*input)[selectedPointIndex];
    int r = selectedPoint.r;
    int g = selectedPoint.g;
    int b = selectedPoint.b;
    int i = 0;
    for (i = 0; i < input->size(); i++)
    {
        if
        (
             abs(input->at(i).r-r)<threshold &&
             abs(input->at(i).g-g)<threshold &&
             abs(input->at(i).b-b)<threshold
        )
            output->indices.push_back(i);
    }
    cout << "OK!"<<endl;
}

void
colorIndices
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr indices
)
{
    cout << "colorIndices..." << flush;
    cout << indices->indices.back()<<endl;
    while (! indices->indices.empty())
    {
        int pointN= indices->indices.back();
        input->at(pointN).r=0;
        input->at(pointN).g=255;
        input->at(pointN).b=0;
        indices->indices.pop_back();
    }
    cout << "OK!"<<endl;
}
