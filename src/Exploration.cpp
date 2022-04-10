#include <Exploration.h>


using namespace std;
class Exploration
{
public:
    cv::Mat MapMat,  FreeRegionMat, SkeletonMat;
    float resolution;

    AutoRun AR;
    vector<Point> ends_of_skeleton;
    vector<MyNode> nodes_of_skeleton;
    vector<EndToNodeChain *> end_to_node_chain;
    vector<NodeToNodeChain *> node_to_node_chain;
    Mat ConnectionMat;
    
    bool IsGraphUpdated, IsMapUpdated;



public:
    Exploration()
    {
        
        IsGraphUpdated = false;
        IsMapUpdated = false;


        MapMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(127));
        FreeRegionMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));
        SkeletonMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));


    }

    void buildGraph(Mat MapMat)
    {
        
        // *********************
        // **** build graph ****
        // *********************
        Mat MapMat_copy = MapMat.clone();    // your OGM
        
        // extract free area map, freeRegionMat
        cout << "Generating SFAM" << endl;
        Mat freeRegionMat_tmp = AR.freeRegionExtraction(MapMat_copy);
        cout << "Finished generating SFAM" << endl;
        cv::imshow("SFAM", freeRegionMat_tmp);
        waitKey(0);

        // extract RAGVD, skeletonMat
        cout << "Generating skeleton" << endl;
        Mat skeleton_tmp = AR.skeletonExtraction(freeRegionMat_tmp);
        cout << "Finished generating skeleton" << endl;
        cv::imshow("RAGVD", skeleton_tmp);
        waitKey(0);

        // extract RAGVG
        cout << "Generating RAGVG" << endl;
        Mat outputRAGVG = AR.RAGVGExtraction(skeleton_tmp, ends_of_skeleton, nodes_of_skeleton, end_to_node_chain, node_to_node_chain, ConnectionMat);
        cout << "Finished generating RAGVG" << endl;
        cv::imshow("RAGVG", outputRAGVG);
        waitKey(0);

        FreeRegionMat.release();
        FreeRegionMat = freeRegionMat_tmp.clone();
        SkeletonMat.release();
        SkeletonMat = skeleton_tmp.clone();
        
        
    }
};





int main(int argc, char** argv)
{
    Mat map = cv::imread("/home/jsonglaptop/catkin_ws/src/RAGVG/maps/scott_hall_edit.png", IMREAD_GRAYSCALE); // your OGM
    cout << "Map is " << map.cols << " x " << map.rows << endl;
    cv::imshow("Provided map", map);
    waitKey(0);
    Exploration exp;
    exp.MapMat = map.clone();
    exp.buildGraph(exp.MapMat);
    return 0;
}