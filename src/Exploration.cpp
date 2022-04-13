#include <Exploration.h>
// geometry_msgs::PointStamped output;
std::vector<geometry_msgs::PointStamped> output_nodes;
using namespace std;
#include <visualization_msgs/Marker.h>
#include "robosar_messages/taskgen_getwaypts.h"

#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
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
        for (auto j: ends_of_skeleton)
        {
            
                // std::cout<<"Point number"<<k<<"::X::"<<j.x<<"y::"<<j.y<<"\n";
                p.point.x = (int)j.y;
                p.point.y = (int)j.x;
                output_nodes.push_back(p);
        }

        // for (auto o:output_nodes)
        // {
        //     int i=1;
        //     std::cout<<"OUTPUT NODE NUMBER::"<<i<<":::"<<o.point.x<<":::"<<o.point.y<<"\n";
        //     i++;
        // }
        cout << "Finished generating RAGVG" << endl;
        cv::imwrite(out_dir+"4_RAGVG.png", outputRAGVG);
        cv::imshow("RAGVG", outputRAGVG);
        waitKey(0);

        FreeRegionMat.release();
        FreeRegionMat = freeRegionMat_tmp.clone();
        SkeletonMat.release();
        SkeletonMat = skeleton_tmp.clone();
        
    }

   
};

bool pubtasks(robosar_messages::taskgen_getwaypts::Request  &req, robosar_messages::taskgen_getwaypts::Response &res)
{
    Mat map_gen = cv::imread("/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_hall_PR4.png", IMREAD_GRAYSCALE); // your OGM
    // int rows = req.map.info.height;
    // int cols = req.map.info.width;
    // float resolution= req.map.info.resolution;
    // float origin_x = req.map.info.origin.position.x;
    // float origin_y = req.map.info.origin.position.y;
    // std::vector<signed char> input = req.map.data;
    // int it = 0;
    // int array[rows][cols];
    // Mat map_gen = cv::Mat::zeros(cv::Size(rows,cols), CV_8UC1);

    // cout<<"finished populating input \n";
    // for(int i = 0; i < map_gen.rows; i++)
    // {
    //     unsigned char* row_ptr = map_gen.ptr<unsigned char>(i);
    //     cout<<"got row ptr \n";
    //     for(int j = 0; j < map_gen.cols; j++)
    //     {
    //         cout<<"before accessing pixel value \n";
    //         row_ptr[j] = (unsigned char)input[it];
    //         cout<<"row value::"<<i<<":::column value::"<<j<<"::::after accessing pixel value:::"<<row_ptr[j]<<":::array value::"<<(unsigned char)input[it]<<"\n";
    //         it++;
    //     }
    // }
    cout << "Map is " << map_gen.cols << " x " << map_gen.rows << "\n";
    // cout<"first value:::"<<map[0][0];
    cv::imwrite(out_dir+"updated_1_OGM.png", map_gen);
    cv::imshow("Provided map", map_gen);
    waitKey(0);
    Exploration exp;
    exp.MapMat = map_gen.clone();
    exp.buildGraph(exp.MapMat);
    geometry_msgs::PointStamped p;
    std::vector<long int> waypts;
    waypts.resize(output_nodes.size());
    int i = 0;
    for (auto o:output_nodes)
    {    
        p.point.x = o.point.x;
        p.point.y = o.point.y;
        int px = (int)o.point.x;
        int py = (int)o.point.y;
        // if ((px == 310 && py == 143) || (px == 182&& py==114) || (px==231 && py == 47))
        // {
        //     continue;
        // }

        waypts[i] = px;
        i+=1;
        waypts[i] = py;
        i+=1;
        // gen_pub.publish(p);
    }
    res.waypoints = waypts;
    res.num_pts = waypts.size();
    res.dims= 2;
    return true;
}

// 310:143
// 182:114
// 231:47

// int main(int argc, char** argv)
// {

//     ros::init(argc, argv, "gen");
//     ros::NodeHandle n;
//     ros::Publisher gen_pub = n.advertise<geometry_msgs::PointStamped>("generated_points", 10);
//     nh.advertiseService("task_generator", pubtasks);
//     Mat map = cv::imread("/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_hall_PR4.png", IMREAD_GRAYSCALE); // your OGM
//     cout << "Map is " << map.cols << " x " << map.rows << endl;
//     cv::imwrite(out_dir+"1_OGM.png", map);
//     cv::imshow("Provided map", map);
//     waitKey(0);
//     Exploration exp;
//     exp.MapMat = map.clone();
//     exp.buildGraph(exp.MapMat);
//     geometry_msgs::PointStamped p;
//     for (auto o:output_nodes)
//     {    
//         p.point.x = o.point.x;
//         p.point.y = o.point.y;
//         gen_pub.publish(p);
//     }
//     bool pubtasks(robosar_messages::taskgen_getwaypts::Request  &req, robosar_messages::taskgen_getwaypts::Response &res)
//     {
//         std::vector<std_msgs::Int64> waypoints;
//         int i = 0;
//         for (auto o:output_nodes)
//         {    
//             p.point.x = o.point.x;
//             p.point.y = o.point.y;
//             waypoints.at(i) = o.point.x;
//             i+=1;
//             waypoints.at(i) = o.point.y;
//             i+=1;
//             // gen_pub.publish(p);
//         }
//         res.waypoints = waypoints;
//         res.num_pts = output_nodes.size();
//         res.dims= 2;
//         return true;
//     }
//     return 0;
// }

int main(int argc, char** argv)
{
    ros::init(argc,argv, "gen");
    ros::NodeHandle nh;
    // Exploration exp;
    ros::ServiceServer sh;
    sh = nh.advertiseService("taskgen_getwaypts", pubtasks);
    ros::spin();
    return 0;
}