#include "componentSelection.h"

void
segmentCluster
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    double threshold
)
{
    cout << "segmentCluster... " << flush; // DEBUG
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
    int selectedClusterIndex=-1;
    for (int i=0;i<cluster_indices_out.size();i++)
        if (selectedClusterIndex>0) break;
        else
            for (int j=0;j<cluster_indices_out[i].indices.size();j++)
                if(cluster_indices_out[i].indices.at(j)==selectedPointIndex)
                {
                    selectedClusterIndex=i;
                    break;
                }

    //non sono riuscito a tirar fuori il cluster in altro modo, ma si dovrebbe riuscire a fare meglio...
    pcl::PointIndices temp=cluster_indices_out[selectedClusterIndex];
    while(!temp.indices.empty())
    {
        output->indices.push_back(temp.indices.back());
        temp.indices.pop_back();
    }
}

void segmentColor
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    int threshold
)
{
    cout << "segmentColor... " << flush; // DEBUG
    pcl::PointXYZRGB selectedPoint = (*input)[selectedPointIndex];
    int r = selectedPoint.r;
    int g = selectedPoint.g;
    int b = selectedPoint.b;
    int i = 0;
    for (i = input->size()-1; i >0 ; i--)//Al contrario per mantenere ordinati in modo crescente gli indici
    {
        if
        (
             abs(input->at(i).r-r)<threshold &&
             abs(input->at(i).g-g)<threshold &&
             abs(input->at(i).b-b)<threshold
        )
        output->indices.push_back(i);

    }

}


void
intersectIndices
(
    pcl::PointIndices::Ptr first,
    pcl::PointIndices::Ptr second,
    pcl::PointIndices::Ptr intersection
)
{
    cout << "intersectIndices... " << flush; // DEBUG

    // point indices intersection cycle
    while(!first->indices.empty() && !second->indices.empty())
    {
        if(first->indices.back()==second->indices.back())//trovato
        {
            intersection->indices.push_back(first->indices.back()); //pusha dentro
            first->indices.pop_back(); //passa al prossimo
        }
        else //poppa fuori il minimo
        {
            if(first->indices.back()  <  second->indices.back())
                first->indices.pop_back();
            else
                second->indices.pop_back();
        }
    }

}



