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
#include <ros/package.h>

std::string output_dir = ros::package::getPath("robosar_ragvg");
using namespace std;

string out_dir = output_dir + "/output/";

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


    vector<geometry_msgs::PointStamped> output_nodes;
    int sliding_window_width = 30;
    int interp_width = 1; //n on either side



    Exploration()
    {
        
        IsGraphUpdated = false;
        IsMapUpdated = false;


        MapMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(127));
        FreeRegionMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));
        SkeletonMat = cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));


    }

void OutputEndsOfSkeleton(vector<Point> ends_of_skeleton, vector<geometry_msgs::PointStamped> output_nodes, geometry_msgs::PointStamped p)
    {
        for (auto e:ends_of_skeleton)
        {
            int rows = 700;
            p.point.x = (int)(e.y);
            p.point.y = rows - (int)e.x;
            int flag = 0;
            cout<<"row::"<<p.point.x<<"::col::"<<p.point.y<<"\n";
            for (auto i:output_nodes)
            {
                if ((abs(i.point.x-p.point.x)<=sliding_window_width) &&(abs(i.point.y-p.point.y)<=sliding_window_width))
                {    
                    flag = 1;
                }
                        
            }
            
        }
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
        // cv::imshow("SFAM", freeRegionMat_tmp);
        // waitKey(0);

        // extract RAGVD, skeletonMat
        cout << "Generating skeleton" << endl;
        Mat skeleton_tmp = AR.skeletonExtraction(freeRegionMat_tmp);
        cout << "Finished generating skeleton" << endl;
        
        cv::imwrite(out_dir+"3_RAGVD.png", skeleton_tmp);
        // cv::imshow("RAGVD", skeleton_tmp);
        // waitKey(0);

        // extract RAGVG
        cout << "Generating RAGVG" << endl;
        Mat outputRAGVG = AR.RAGVGExtraction(skeleton_tmp, ends_of_skeleton, nodes_of_skeleton, end_to_node_chain, node_to_node_chain, ConnectionMat, output_nodes, rows, sliding_window_width, interp_width);
        
        std::cout<<"output nodes size:::"<<output_nodes.size();
        for (auto j: nodes_of_skeleton)
        {
               
                for (auto point:j.points)
                {
  
                    p.point.x =  (int)point.y;
                    p.point.y =   rows - (int)point.x;
                
                    int flag = 0;
                    for (auto i:output_nodes)
                    {
                        if ((abs(i.point.x-p.point.x)<=sliding_window_width) &&(abs(i.point.y-p.point.y)<=sliding_window_width))
                        {    
                         
                            flag = 1;
                        }
                        
                    }
                    if (flag ==0)
                    {    
                        if (p.point.x>=565 || p.point.x<=90) {
                            continue;
                        }
                        // if(((p.point.x == 66) && (p.point.y == 29)) || ((p.point.x == 473) && (p.point.y == 139)))
                        // {
                        //     cout<<"inside removal condition!!!!! \n";
                        //     continue;
                        // }
                        output_nodes.push_back(p);
                    }
                       
                }
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

    void refineNodes(int r, Mat& MapMat) {
        vector<geometry_msgs::PointStamped> refined_nodes;
        for (auto o : output_nodes) {
            int x = (int)o.point.x;
            int y = (int)o.point.y;
            cout << x << " " << y << endl;
            printf("%hhu, ", MapMat.at<int8_t>(x,y));
            cout << endl;
            if (!checkCollision(x, y, r, MapMat)) {
                refined_nodes.push_back(o);
            }
        }
        output_nodes = refined_nodes;
    }

    bool checkCollision(int x, int y, int r, Mat& MapMat) {
        for (int i=-r; i<=r; i++) {
            for (int j=-r; j<=r; j++) {
                if (x+i>=0 && x+i<MapMat.rows && y+j>=0 && y+j<MapMat.cols) {
                    if (MapMat.at<int8_t>(x+i,y+j)==0) {  
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
    // Mat map_gen = cv::imread("/home/naren/Downloads/scott_hall_SVD.pgm", IMREAD_GRAYSCALE); // your OGM
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
            map_gen.at<int8_t>(i,j) = temp_val;
            // printf("%hhu, ", map_gen.at<int8_t>(i,j));
            // cout<<map_gen.at<int8_t>(i,j)<<",";
            // cout<<"row value::"<<i<<":::column value::"<<j<<"::::after accessing pixel value:::"<<row_ptr[j]<<":::array value::"<<input[it]<<"\n";
            it++;
        }
    }
    cout << "Map is " << map_gen.cols << " x " << map_gen.rows << "\n";
    cv::flip(map_gen, map_gen, 0);
    cv::imwrite(out_dir+"updated_1_OGM.png", map_gen);
    cv::imshow("Provided map", map_gen);
    waitKey(0);
    Exploration exp;
    exp.MapMat = map_gen.clone();
    exp.buildGraph();
    // exp.refineNodes(3, map_gen);
    geometry_msgs::PointStamped p;
    std::vector<long int> waypts;
    int i = 0;
    for (auto o:exp.output_nodes)
    {    

        int px = (int)o.point.x;
        int py = (int)o.point.y;
        cout<<"["<<px<<","<<py<<"],";
        waypts.push_back(px);
        waypts.push_back(py);
    }
    res.waypoints = waypts;
    res.num_pts = waypts.size();
    cout << "Num waypts = " << res.num_pts << endl;
    res.dims= 2;
    return true;
    
}

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