#include <Exploration.h>
#include <visualization_msgs/Marker.h>
#include "robosar_messages/taskgen_getwaypts.h"
#include <algorithm> 
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "quadtree.h"
#include <math.h>

string out_dir = "/home/charvi/Documents/robosar/src/robosar_ragvg/output/";
int prohibited_x = 0;
int prohibited_y = 0;
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

    // geometry_msgs::PointStamped output;
    vector<geometry_msgs::PointStamped> output_nodes;
    int sliding_window_width = 20;
    int interp_width = 1; //n on either side


    // ros::init(argc, argv, "nodes_publisher");

    Exploration()
    {
        
        IsGraphUpdated = false;
        IsMapUpdated = false;


        MapMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(127));
        FreeRegionMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));
        SkeletonMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));


    }

    void buildGraph()
    {
        geometry_msgs::PointStamped p;
        int cols = MapMat.cols;
        int rows = MapMat.rows;
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
        Mat outputRAGVG = AR.RAGVGExtraction(skeleton_tmp, ends_of_skeleton, nodes_of_skeleton, end_to_node_chain, node_to_node_chain, ConnectionMat, output_nodes, rows, sliding_window_width, interp_width);
        // std::cout<<nodes_of_skeleton.size()<<"\n";
        
        std::cout<<"output nodes size:::"<<output_nodes.size();
        for (auto j: nodes_of_skeleton)
        {
                std::cout<<"inside for loop nodes of skeleton block \n";
               
                for (auto point:j.points)
                {
                    // cout<<point.x<<"\n"
                    p.point.x =  (int)point.y;
                    p.point.y =   rows - (int)point.x;
                    // flag = 0;
                    // // cout<<"row::"<<p.point.x<<"::col::"<<p.point.y<<"\n";
                    // for (auto i:output_nodes)
                    // {
                    //     if ((i.point.x == p.point.x) && (i.point.y == p.point.y))
                    //     {
                    //         continue;
                    //     }
                    //     else
                    //     {
                    //         output_nodes.push_back(p);
                    //     }
                    // }
                    int flag = 0;
                    for (auto i:output_nodes)
                    {
                        if ((abs(i.point.x-p.point.x)<=sliding_window_width) &&(abs(i.point.y-p.point.y)<=sliding_window_width))
                        {    
                            cout<<"flag 1 condition reached \n";
                            cout<<"values::::output_nodes::"<<i.point.x<<" "<<i.point.y<<"\n";
                            cout<<"nodes of skeleton::"<<p.point.x<<" "<<p.point.y<<"\n";
                            flag = 1;
                        }
                        
                    }
                    if (flag ==0)
                    {    
                            if(p.point.x>=640)
                            {
                                continue;
                            }
                            if(((p.point.x == 445) && (p.point.y == 143)) || ((p.point.x == 473) && (p.point.y == 139)))
                            {
                                cout<<"inside removal condition!!!!! \n";
                                continue;
                            }
   
                        output_nodes.push_back(p);
                    }
                }
                // int px = j.points;
                // int a = px.x;
    
                // // p.point.x = (int)j.points.;
                // // p.point.y = (int)j.points.data.y;
                // output_nodes.push_back(p);
        }
        // for (auto e:ends_of_skeleton)
        // {
        //     p.point.x = (int)(e.y);
        //     p.point.y = rows - (int)e.x;
        //     flag = 0;
        //     cout<<"row::"<<p.point.x<<"::col::"<<p.point.y<<"\n";
        //     for (auto i:output_nodes)
        //     {
        //         if ((abs(i.point.x-p.point.x)<=sliding_window_width) &&(abs(i.point.y-p.point.y)<=sliding_window_width))
        //         {    
        //             // cout<<"flag 1 condition reached \n";
        //             // cout<<"values::::output_nodes::"<<i.point.x<<" "<<i.point.y<<"\n";
        //             // cout<<"nodes of skeleton::"<<p.point.x<<" "<<p.point.y<<"\n";
        //             flag = 1;
        //         }
                        
        //     }
            
        //     if (flag ==0)
        //     {    
        //         // output_nodes.push_back(p);
        //     }
            
        //     // output_nodes.push_back(p);
        // }
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

        cout << MapMat.rows << " " << MapMat.cols << endl;
        refineNodes(4);
        
    }

    void refineNodes(int r) {
        vector<geometry_msgs::PointStamped> refined_nodes;
        for (auto o : output_nodes) {
            int x = (int)o.point.x;
            int y = (int)o.point.y;
            if (!checkCollision(x, y, r)) {
                refined_nodes.push_back(o);
            }
        }
        output_nodes = refined_nodes;
    }

    bool checkCollision(int x, int y, int r) {
        for (int i=-r; i<=r; i++) {
            for (int j=-r; j<=r; j++) {
                if (x+i>=0 && x+i<MapMat.cols && y+j>=0 && y+j<MapMat.rows) {
                    if (y==126) {
                        cout << x+i << " " << y+j << " ";
                        printf("%hhu, ", MapMat.at<int8_t>(x+i,y+j));
                        cout << endl;
                    }
                    if (MapMat.at<int8_t>(x+i,y+j)>=254) {  
                        return true;
                    }
                } 
            }
        }
        return false;
    }

   
};

bool pubtasks(robosar_messages::taskgen_getwaypts::Request  &req, robosar_messages::taskgen_getwaypts::Response &res)
{
    // Mat map_gen = cv::imread("/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_final.png", IMREAD_GRAYSCALE); // your OGM
    int rows = req.map.info.height;
    int cols = req.map.info.width;
    float resolution= req.map.info.resolution;
    float origin_x = req.map.info.origin.position.x;
    float origin_y = req.map.info.origin.position.y;

    // int8_t a[rows*cols]
    cout<<"service triggered!!!!! \n";

    Mat map_gen = cv::Mat::zeros(cv::Size(cols,rows), CV_8UC1);
    int it = 0;
    int temp_val;
    cout << endl;
    for(int i = 0; i < map_gen.rows; i++)
    {
        // int8_t* row_ptr = map_gen.ptr<int8_t>(i);
        // cout<<"got row ptr \n";
        for(int j = 0; j < map_gen.cols; j++)
        {
            temp_val = req.map.data[it];

            if(temp_val < 0){
                temp_val = 0;
            }else if(temp_val > 50){
                temp_val = 0;
            }else if(temp_val <= 50){
                temp_val = 254;
            }
            // row_ptr[j] = req.map.data[it];
            map_gen.at<int8_t>(i,j) = temp_val;
            // printf("%hhu, ", map_gen.at<int8_t>(i,j));
            // cout<<map_gen.at<int8_t>(i,j)<<",";
            // cout<<"row value::"<<i<<":::column value::"<<j<<"::::after accessing pixel value:::"<<row_ptr[j]<<":::array value::"<<input[it]<<"\n";
            it++;
        }
    }
    cout << "Map is " << map_gen.cols << " x " << map_gen.rows << "\n";
    // cout<"first value:::"<<map[0][0];
    cv::flip(map_gen, map_gen, 0);
    cv::imwrite(out_dir+"updated_1_OGM.png", map_gen);
    cv::imshow("Provided map", map_gen);
    waitKey(0);
    Exploration exp;
    exp.MapMat = map_gen.clone();
    exp.buildGraph();
    geometry_msgs::PointStamped p;
    std::vector<long int> waypts;
    // waypts.resize(output_nodes.size());
    int i = 0;
    for (auto o:exp.output_nodes)
    {    
        // p.point.x = o.point.x;
        // p.point.y = o.point.y;

        int px = (int)o.point.x;
        int py = (int)o.point.y;
        // if (px==496 && py==2)
        // {
        //     continue;
        // }
        // if ((px == 310 && py == 143) || (px == 182&& py==114) || (px==231 && py == 47))
        // {
        //     continue;
        // }
        cout<<"["<<px<<","<<py<<"],";
        waypts.push_back(px);
        waypts.push_back(py);
        // gen_pub.publish(p);
    }
    res.waypoints = waypts;
    res.num_pts = waypts.size();
    cout << "Num waypts = " << res.num_pts << endl;
    res.dims= 2;
    // std::vector<int> iters_to_erase ;
    // int iterator =0;
    // int distance_flag = 0;
    // int equal_flag = 0;
    // for (auto o:output_nodes)
    // {
    //     distance_flag = 0;
    //     equal_flag = 0;
    //     for(auto k:output_nodes)
    //     {
    //         if ((o.point.x == k.point.x) && (o.point.y == k.point.y))
    //         {
    //             equal_flag = 1;
    //             continue;
    //         }
            
    //         if(((o.point.x - k.point.x)<2) && ((o.point.y-k.point.y)<2))
    //         {
    //             distance_flag = 1;
    //         }
    //     }
    //     if(equal_flag==1)
    //     {
    //         continue;
    //     }
    //     else
    //     {
    //         if(distance_flag==1)
    //         {
    //             iters_to_erase.push_back(iterator);
    //             // output_nodes = remove(output_nodes.begin(),output_nodes.end(),o);
    //         }
    //     }
    //     iterator++;
    // }
    // if(output_nodes.empty() == false) 
    // {
    //     for(int i = output_nodes.size() - 1; i >= 0; i--)
    //     {
    //         if(condition) 
    //             vector_name.erase(vector_name.at(i));
    //     }
    // }
    // for(auto i:iters_to_erase)
    // {
    //     int it = output_nodes.begin
    //     output_nodes.erase(i);
    // }
    
 
    cout<<"No seg fault till here \n";
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