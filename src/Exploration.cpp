#include <Exploration.h>
// geometry_msgs::PointStamped output;
std::vector<geometry_msgs::PointStamped> output_nodes;
using namespace std;
#include <visualization_msgs/Marker.h>
#include "robosar_messages/auto_taskgen_getwaypts.h"
string out_dir = "/home/naren/catkin_ws/src/robosar_ragvg/output/";

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
    // bool pubWaypoints(robosar_messages::auto_taskgen_getwaypts::Request  &req, robosar_messages::auto_taskgen_getwaypts::Response &res);
    Mat ConnectionMat;
    
    bool IsGraphUpdated, IsMapUpdated;

    // ros::init(argc, argv, "nodes_publisher");

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
        geometry_msgs::PointStamped p;
        // *********************
        // **** build graph ****
        // *********************
        Mat MapMat_copy = MapMat.clone();    // your OGM
        
        // extract free area map, freeRegionMat
        cout << "Generating SFAM" << endl;
        Mat freeRegionMat_tmp = AR.freeRegionExtraction(MapMat_copy);
        cout << "Finished generating SFAM" << endl;
        cv::imwrite(out_dir+"2_SFAM.png", freeRegionMat_tmp);
        cv::imshow("SFAM", freeRegionMat_tmp);
        waitKey(0);

        // extract RAGVD, skeletonMat
        cout << "Generating skeleton" << endl;
        Mat skeleton_tmp = AR.skeletonExtraction(freeRegionMat_tmp);
        cout << "Finished generating skeleton" << endl;
        
        cv::imwrite(out_dir+"3_RAGVD.png", skeleton_tmp);
        cv::imshow("RAGVD", skeleton_tmp);
        waitKey(0);

        // extract RAGVG
        cout << "Generating RAGVG" << endl;
        Mat outputRAGVG = AR.RAGVGExtraction(skeleton_tmp, ends_of_skeleton, nodes_of_skeleton, end_to_node_chain, node_to_node_chain, ConnectionMat);
        std::cout<<nodes_of_skeleton.size()<<"\n";
        for (auto i: nodes_of_skeleton)
        {
            for (auto j: i.points)
            {
                // std::cout<<"Point number"<<k<<"::X::"<<j.x<<"y::"<<j.y<<"\n";
                p.point.x = j.x;
                p.point.y = j.y;
                output_nodes.push_back(p);
            }
        }

        for (auto o:output_nodes)
        {
            int i=1;
            std::cout<<"OUTPUT NODE NUMBER::"<<i<<":::"<<o.point.x<<":::"<<o.point.y<<"\n";
            i++;
        }
        cout << "Finished generating RAGVG" << endl;
        cv::imwrite(out_dir+"4_RAGVG.png", outputRAGVG);
        cv::imshow("RAGVG", outputRAGVG);
        waitKey(0);

        FreeRegionMat.release();
        FreeRegionMat = freeRegionMat_tmp.clone();
        SkeletonMat.release();
        SkeletonMat = skeleton_tmp.clone();
        
    }

    // bool pubWaypoints(robosar_messages::auto_taskgen_getwaypts::Request  &req, robosar_messages::auto_taskgen_getwaypts::Response &res)
    // {
    //     std::vector<geometry_msgs::PointStamped> output_values;
    //     for (auto i:output_nodes)
    //         output_values.push_back(i);
    //     res.waypoints = output_values;
    //     return true;
    // }
};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "gen");
    ros::NodeHandle n;
    ros::Publisher gen_pub = n.advertise<geometry_msgs::PointStamped>("generated_points", 10);
    Mat map = cv::imread("/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_hall_plus_edit.png", IMREAD_GRAYSCALE); // your OGM
    cout << "Map is " << map.cols << " x " << map.rows << endl;
    cv::imwrite(out_dir+"1_OGM.png", map);
    cv::imshow("Provided map", map);
    waitKey(0);
    Exploration exp;
    exp.MapMat = map.clone();
    exp.buildGraph(exp.MapMat);
    geometry_msgs::PointStamped p;
    for (auto o:output_nodes)
    {    
        p.point.x = o.point.x;
        p.point.y = o.point.y;
        gen_pub.publish(p);

    }
    return 0;
}