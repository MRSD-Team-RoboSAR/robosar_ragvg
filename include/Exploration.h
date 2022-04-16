#include <Eigen/Eigen>
#include "my_config.h"
#include "KMM.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <geometry_msgs/PointStamped.h> 
#include <ros/ros.h>
#include "robosar_messages/auto_taskgen_getwaypts.h"
using namespace cv;
using namespace std;


double DistanceBetweenPoints(Point pt1, Point pt2)
{
    return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}


#pragma region RAGVG

// node
class MyNode
{
public:
	vector<Point> points;
	MyNode(){};
	MyNode(const MyNode &node)
	{
		int size = node.points.size();
        points = vector<Point>(node.points);
	}
    MyNode& operator=(const MyNode &node)
    {
        points.clear();
        points = vector<Point>(node.points);
        return *this;
    }

	bool IsReachNode(Point pt)
	{
		bool IsReach = false;
		for (vector<Point>::iterator it = points.begin(); it != points.end(); it++)
		{
			if (abs(pt.x - it->x) <= 1 && abs(pt.y - it->y) <= 1)
			{
				IsReach = true;
				return IsReach;
			}
		}
		return IsReach;
	}

    float distanceToNode(MyNode node)
    {
        float dist = 100.0;
        for(auto p1:points)
        {
            for(auto p2:node.points)
            {
                float tmp_dist = DistanceBetweenPoints(p1, p2);
                dist = tmp_dist < dist ? tmp_dist : dist;
            }
        }
        return dist;
    }

	void drawNode(Mat src, Mat &dst, int gray)
	{
		dst = src.clone();
		for (vector<Point>::iterator it = points.begin(); it != points.end(); it++)
		{
            dst.at<uchar>(it->x, it->y) = gray;
		}
	}
	
	Point centerPoint()
	{
		double x = 0.0, y = 0.0;
		for (vector<Point>::iterator it = points.begin(); it != points.end(); it++)
		{
			x += it->x;
			y += it->y;
		}
		int size = points.size();
		return Point(floor(x / (double)size), floor(y / (double)size));
	}

	bool operator==(MyNode node)
	{
		if (this->points.size() == node.points.size() && DistanceBetweenPoints(this->centerPoint(), node.centerPoint()) < 5)
			return true;
		else
			return false;
	}
};

// branch
struct EndToNodeChain
{
	Point m_End;
	MyNode m_Node;
	int index_end, index_node;
    bool IsReverse;
	vector<Point> Chain;
    float length;
public:
	EndToNodeChain(Point end)
	{
		m_End = Point(end);
	}
	void draw(Mat src, Mat &dst, Scalar scalar)
	{
		dst = src.clone();
		vector<Point> tmp_chain;
		for (vector<Point>::iterator it = Chain.begin(); it != Chain.end(); it++)
		{
			Point pt = *it;
			tmp_chain.push_back(Point(pt.y, pt.x));
		}
		polylines(dst, tmp_chain, false, scalar, 1);
	}
    int size()
	{
		return Chain.size();
	}
    float Length()
    {
        float res = 0;
        Point last_pt(-1,-1);
        for(auto pt:Chain)
        {
            if(last_pt.x == -1 && last_pt.y == -1)
            {
                last_pt = Point(pt);
                continue;
            }
            else
            {
                if(abs(pt.x - last_pt.x) + abs(pt.y - last_pt.y) == 1)
                    res += 1.0;
                else
                    res += 1.414;
            }
        }
        return res;
    }
};

// link
struct NodeToNodeChain
{
	MyNode m_Node1, m_Node2;
	int m_index1, m_index2;
	vector<Point> Chain;
    float length;
public:
	NodeToNodeChain(MyNode node)
	{
		m_Node1 = MyNode(node);
	}
    NodeToNodeChain()
	{
	}
	vector<Point> draw(Mat src, Mat &dst, Scalar scalar)
	{
		dst = src.clone();
		vector<Point> tmp_chain;
		for (vector<Point>::iterator it = Chain.begin(); it != Chain.end(); it++)
		{
			Point pt = *it;
			tmp_chain.push_back(Point(pt.y, pt.x));
		}
        
		polylines(dst, tmp_chain, false, scalar, 1);
        return tmp_chain;
    }
	int size()
	{
		return Chain.size();
	}
    float Length()
    {
        float res = 0;
        Point last_pt(-1,-1);
        for(auto pt:Chain)
        {
            if(last_pt.x == -1 && last_pt.y == -1)
            {
                last_pt = Point(pt);
                continue;
            }
            else
            {
                if(abs(pt.x - last_pt.x) + abs(pt.y - last_pt.y) == 1)
                    res += 1.0;
                else
                    res += 1.414;
            }
        }
        return res;
    }
	bool operator==(NodeToNodeChain chain)
	{
		if (m_Node1 == chain.m_Node1 && m_Node2 == chain.m_Node2)
			return true;
		if (m_Node2 == chain.m_Node1 && m_Node1 == chain.m_Node2)
			return true;
		return false;
	}
};

#pragma endregion RAGVG





class AutoRun
{
public:
    Mat map;

    vector<Mat> element_vector;

    // params from .ini file
    int obstacle_dilate_size;
    int wall_remove_small_region_size;
    int free_dilate_size;
    int image_blur_size;
    int image_blur_threshold;
    int delete_jut_uthreshold;
    int delete_jut_vthreshold;

    AutoRun()
    {
        string root = "/home/rachelzheng/robosar_ws/src/robosar_ragvg/src/";  // To be changed
        map = Mat::zeros(Size(2,2), CV_8UC1);
        for(int i = 1; i <=7; i++)
            element_vector.push_back(Mat(i, i, CV_8U, Scalar(1)));
        readIniFile(root + "params.txt");
    }

    void readIniFile(string filename)
    {
        Config config(filename);
        obstacle_dilate_size = config.Read("obstacle_dilate_size", 5);
        wall_remove_small_region_size = config.Read("wall_remove_small_region_size", 2);
        free_dilate_size = config.Read("free_dilate_size", 3);
        image_blur_size = config.Read("image_blur_size", 8);
        image_blur_threshold = config.Read("image_blur_threshold", 70);
        delete_jut_uthreshold = config.Read("delete_jut_uthreshold", 3);
        delete_jut_vthreshold = config.Read("delete_jut_vthreshold", 3);
    }

    // extract largest connection area
    void LargestConnecttedComponent(Mat srcImage, Mat &dstImage)
    {
        Mat temp;
        Mat labels;
        srcImage.copyTo(temp);

        //1. 标记连通域
        int n_comps = connectedComponents(temp, labels, 4, CV_16U);
        vector<int> histogram_of_labels;
        for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
        {
            histogram_of_labels.push_back(0);
        }

        int rows = labels.rows;
        int cols = labels.cols;
        for (int row = 0; row < rows; row++) //计算每个labels的个数
        {
            unsigned short * data = labels.ptr<ushort>(row);
            for (int col = 0; col < cols; col++)
                histogram_of_labels[data[col]] += 1;
        }
        histogram_of_labels[0] = 0; //将背景的labels个数设置为0

        //2. 计算最大的连通域labels索引
        int maximum = 0;
        int max_idx = 0;
        for (int i = 0; i < n_comps; i++)
        {
            if (histogram_of_labels.at(i) > maximum)
            {
                maximum = histogram_of_labels[i];
                max_idx = i;
            }
        }

        //3. 将最大连通域标记为1
        for (int row = 0; row < rows; row++) 
        {
            unsigned short * data = labels.ptr<ushort>(row);
            for (int col = 0; col < cols; col++)
            {
                if (data[col] == max_idx)
                {
                    data[col] = 255;
                }
                else
                {
                    data[col] = 0;
                }
            }
        }

        //4. 将图像更改为CV_8U格式
        labels.convertTo(dstImage, CV_8UC1);
    }

    // 去除小的连通区域
    void RemoveSmallRegion(Mat& Src, Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode)
    {
        for (int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            for (int j = 0; j < Src.cols; ++j)
            {
                if (iData[j] == 0 || iData[j] == 255) continue;
                else if (iData[j] < 10)
                {
                    iData[j] = 0;
                    //cout<<'#';  
                }
                else if (iData[j] > 10)
                {
                    iData[j] = 255;
                    //cout<<'!';  
                }
            }
        }

        int RemoveCount = 0;       //记录除去的个数  
        //记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查  
        Mat Pointlabel = Mat::zeros(Src.size(), CV_8UC1);

        if (CheckMode == 1)
        {
            for (int i = 0; i < Src.rows; ++i)
            {
                uchar* iData = Src.ptr<uchar>(i);
                uchar* iLabel = Pointlabel.ptr<uchar>(i);
                for (int j = 0; j < Src.cols; ++j)
                {
                    if (iData[j] < 10)
                    {
                        iLabel[j] = 3;
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < Src.rows; ++i)
            {
                uchar* iData = Src.ptr<uchar>(i);
                uchar* iLabel = Pointlabel.ptr<uchar>(i);
                for (int j = 0; j < Src.cols; ++j)
                {
                    if (iData[j] > 10)
                    {
                        iLabel[j] = 3;
                    }
                }
            }
        }

        vector<Point2i> NeihborPos;  //记录邻域点位置  
        NeihborPos.push_back(Point2i(-1, 0));
        NeihborPos.push_back(Point2i(1, 0));
        NeihborPos.push_back(Point2i(0, -1));
        NeihborPos.push_back(Point2i(0, 1));
        if (NeihborMode == 1)
        {
            NeihborPos.push_back(Point2i(-1, -1));
            NeihborPos.push_back(Point2i(-1, 1));
            NeihborPos.push_back(Point2i(1, -1));
            NeihborPos.push_back(Point2i(1, 1));
        }
        else;
        int NeihborCount = 4 + 4 * NeihborMode;
        int CurrX = 0, CurrY = 0;
        //开始检测  
        for (int i = 0; i < Src.rows; ++i)
        {
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < Src.cols; ++j)
            {
                if (iLabel[j] == 0)
                {
                    //********开始该点处的检查**********  
                    vector<Point2i> GrowBuffer;                                      //堆栈，用于存储生长点  
                    GrowBuffer.push_back(Point2i(j, i));
                    Pointlabel.at<uchar>(i, j) = 1;
                    int CheckResult = 0;                                               //用于判断结果（是否超出大小），0为未超出，1为超出  

                    for (int z = 0; z<GrowBuffer.size(); z++)
                    {

                        for (int q = 0; q<NeihborCount; q++)                                      //检查四个邻域点  
                        {
                            CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
                            CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
                            if (CurrX >= 0 && CurrX<Src.cols&&CurrY >= 0 && CurrY<Src.rows)  //防止越界  
                            {
                                if (Pointlabel.at<uchar>(CurrY, CurrX) == 0)
                                {
                                    GrowBuffer.push_back(Point2i(CurrX, CurrY));  //邻域点加入buffer  
                                    Pointlabel.at<uchar>(CurrY, CurrX) = 1;           //更新邻域点的检查标签，避免重复检查  
                                }
                            }
                        }

                    }
                    if (GrowBuffer.size()>AreaLimit) CheckResult = 2;                 //判断结果（是否超出限定的大小），1为未超出，2为超出  
                    else { CheckResult = 1;   RemoveCount++; }
                    for (int z = 0; z<GrowBuffer.size(); z++)                         //更新Label记录  
                    {
                        CurrX = GrowBuffer.at(z).x;
                        CurrY = GrowBuffer.at(z).y;
                        Pointlabel.at<uchar>(CurrY, CurrX) += CheckResult;
                    }
                    //********结束该点处的检查**********  


                }
            }
        }

        CheckMode = 255 * (1 - CheckMode);
        //开始反转面积过小的区域  
        for (int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            uchar* iDstData = Dst.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < Src.cols; ++j)
            {
                if (iLabel[j] == 2)
                {
                    iDstData[j] = CheckMode;
                }
                else if (iLabel[j] == 3)
                {
                    iDstData[j] = iData[j];
                }
            }
        }
    }

    // extract obstacles (wall)
    Mat wallExtraction(Mat mapMat)
    {
        updateMap(mapMat);
        Mat src_clone = mapMat.clone();
        Mat black;
        threshold(src_clone, black, 50, 255, THRESH_BINARY_INV);
        Mat wall = Mat::zeros(mapMat.size(), CV_8UC1);
        dilate(black, black, element_vector[obstacle_dilate_size]);
        RemoveSmallRegion(black, wall, wall_remove_small_region_size * 10, 1, 1);
        return wall;
    }

    // extract ground (floor)
    Mat groundExtraction(Mat mapMat)
    {
        updateMap(mapMat);
        Mat white = Mat::zeros(mapMat.size(), CV_8UC1);
        cv::threshold(mapMat, white, 150, 255, THRESH_BINARY);

        Mat dilate_white = Mat::zeros(mapMat.size(), CV_8UC1);
        cv::dilate(white, dilate_white, element_vector[free_dilate_size]);

        Mat erode_white = Mat::zeros(mapMat.size(), CV_8UC1);
        cv::erode(dilate_white, erode_white, element_vector[free_dilate_size]);

        Mat clear_area = Mat::zeros(mapMat.size(), CV_8UC1);
        RemoveSmallRegion(erode_white, clear_area, wall_remove_small_region_size * 10, 1, 1);
        return clear_area;
    }

    // extract free area
    Mat freeRegionExtraction(Mat mapMat)
    {
        updateMap(mapMat);
        Mat wall = Mat::zeros(mapMat.size(), CV_8UC1);
        wall = wallExtraction(mapMat);
        Mat clear_area = Mat::zeros(mapMat.size(), CV_8UC1);
        clear_area = groundExtraction(mapMat);

        Mat gray128_255;
        cv::threshold(mapMat, gray128_255, 127, 255, THRESH_TOZERO);

        Mat gray128_250;
        cv::threshold(gray128_255, gray128_250, 200, 255, THRESH_TOZERO_INV);

        Mat gray;
        cv::threshold(gray128_250, gray, 127, 255, THRESH_BINARY);

        Mat true_gray = gray - wall - clear_area;

        Mat dilate_gray;
        cv::dilate(true_gray, dilate_gray, element_vector[free_dilate_size]);

        Mat erode_gray;
        cv::erode(dilate_gray, erode_gray, element_vector[free_dilate_size-1]);

        Mat shadow = Mat::zeros(erode_gray.size(), CV_8UC1);

        Mat shadow_clear_area = (shadow | clear_area) - wall;
        Mat smooth_src1 = Mat::zeros(mapMat.size(), CV_8UC1);
		Mat smooth_src = Mat::zeros(mapMat.size(), CV_8UC1);
        Mat result = Mat::zeros(mapMat.size(), CV_8UC1);
        delete_jut(shadow_clear_area, smooth_src1, delete_jut_uthreshold, delete_jut_vthreshold);
		imageblur(smooth_src1, result, Size(image_blur_size, image_blur_size), image_blur_threshold);   // TODO adjust the parameters
        LargestConnecttedComponent(result, result);
        return result;
    }

    void updateMap(Mat MapMat)
    {
        map = MapMat.clone();
    }

    // remove bulgy sections of rough wall
    //uthreshold、vthreshold can be adjusted
    void delete_jut(Mat& src, Mat& dst, int uthreshold = 3, int vthreshold = 3, int type = 1)
    {
        int threshold;
        src.copyTo(dst);
        int height = dst.rows;
        int width = dst.cols;
        int k;  //用于循环计数传递到外部
        for (int i = 0; i < height - 1; i++)
        {
            uchar* p = dst.ptr<uchar>(i);
            for (int j = 0; j < width - 1; j++)
            {
                if (type == 0)
                {
                    //行消除
                    if (p[j] == 255 && p[j + 1] == 0)
                    {
                        if (j + uthreshold >= width)
                        {
                            for (int k = j + 1; k < width; k++)
                                p[k] = 255;
                        }
                        else
                        {
                            for (k = j + 2; k <= j + uthreshold; k++)
                            {
                                if (p[k] == 255) break;
                            }
                            if (p[k] == 255)
                            {
                                for (int h = j + 1; h < k; h++)
                                    p[h] = 255;
                            }
                        }
                    }
                    //列消除
                    if (p[j] == 255 && p[j + width] == 0)
                    {
                        if (i + vthreshold >= height)
                        {
                            for (k = j + width; k < j + (height - i)*width; k += width)
                                p[k] = 255;
                        }
                        else
                        {
                            for (k = j + 2 * width; k <= j + vthreshold*width; k += width)
                            {
                                if (p[k] == 255) break;
                            }
                            if (p[k] == 255)
                            {
                                for (int h = j + width; h < k; h += width)
                                    p[h] = 255;
                            }
                        }
                    }
                }
                else  //type = 1
                {
                    //行消除
                    if (p[j] == 0 && p[j + 1] == 255)
                    {
                        if (j + uthreshold >= width)
                        {
                            for (int k = j + 1; k < width; k++)
                                p[k] = 0;
                        }
                        else
                        {
                            for (k = j + 2; k <= j + uthreshold; k++)
                            {
                                if (p[k] == 0) break;
                            }
                            if (p[k] == 0)
                            {
                                for (int h = j + 1; h < k; h++)
                                    p[h] = 0;
                            }
                        }
                    }
                    //列消除
                    if (p[j] == 0 && p[j + width] == 255)
                    {
                        if (i + vthreshold >= height)
                        {
                            for (k = j + width; k < j + (height - i)*width; k += width)
                                p[k] = 0;
                        }
                        else
                        {
                            for (k = j + 2 * width; k <= j + vthreshold*width; k += width)
                            {
                                if (p[k] == 0) break;
                            }
                            if (p[k] == 0)
                            {
                                for (int h = j + width; h < k; h += width)
                                    p[h] = 0;
                            }
                        }
                    }
                }
            }
        }
    }

    // smooth the rough wall and corner rounding
    void imageblur(Mat& src, Mat& dst, Size size, int threshold)
    {
        int height = src.rows;
        int width = src.cols;
        blur(src, dst, size);
        for (int i = 0; i < height; i++)
        {
            uchar* p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; j++)
            {
                if (p[j] < threshold)
                    p[j] = 0;
                else p[j] = 255;
            }
        }
    }

    // Free Area Map to RAGVD
    Mat skeletonExtraction(Mat smooth_src)
    {
        Mat skeleton = Mat::zeros(smooth_src.size(), CV_8UC1);
        KMM kmm(smooth_src);
        skeleton = kmm.execute();
        //Mat result = smooth_src - skeleton;
        return skeleton;
    }

    // extract ends
    vector<Point> getEnds(const Mat &thinSrc, int raudis = 1, int thresholdMin = 3)
    {
        assert(thinSrc.type() == CV_8UC1);
        vector<Point> ends;
        int width = thinSrc.cols;
        int height = thinSrc.rows;
        Mat tmp;
        thinSrc.copyTo(tmp);
        vector<Point> points;
        vector<int> counts;

        vector<Point> before_ends;
        vector<int> ends_counts;
                                                                                                                                     
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                if (*(tmp.data + tmp.step * i + j) == 0)
                {
                    continue;
                }
                int count = 0;
                for (int k = i - raudis; k < i + raudis + 1; k++)
                {
                    for (int l = j - raudis; l < j + raudis + 1; l++)
                    {
                        if (k < 0 || l < 0 || k>height - 1 || l>width - 1)
                        {
                            continue;

                        }
                        else if (*(tmp.data + tmp.step * k + l) == 255)
                        {
                            count++;
                        }
                    }
                }

                if (count == thresholdMin-1)
                {
                    //points.push_back(Point(j + x_min * 20, i + y_min * 20));
                    //counts.push_back(count);

                    before_ends.push_back(Point(i, j));
                    ends_counts.push_back(count);

                }
            }
        }

        return before_ends;
    }

    // extract nodes
    vector<MyNode> getNodes(const Mat &thinSrc, unsigned int raudis = 1, unsigned int thresholdMax = 3)
    {
        assert(thinSrc.type() == CV_8UC1);
        vector<MyNode> nodes;
        int width = thinSrc.cols;
        int height = thinSrc.rows;
        Mat tmp;
        thinSrc.copyTo(tmp);

        vector<Point> before_nodes;

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                if (*(tmp.data + tmp.step * i + j) == 0)
                {
                    continue;
                }
                int count = 0;
                for (int k = i - raudis; k < i + raudis + 1; k++)
                {
                    for (int l = j - raudis; l < j + raudis + 1; l++)
                    {
                        if (k < 0 || l < 0 || k>height - 1 || l>width - 1)
                        {
                            continue;

                        }
                        else if (*(tmp.data + tmp.step * k + l) == 255)
                        {
                            count++;
                        }
                    }
                }

                if (count > thresholdMax)
                {
                    before_nodes.push_back(Point(i, j));
                }
            }
        }

        // filter

        int size = before_nodes.size();
        vector<bool> filtered_label(size, false);
        vector<MyNode> origin_nodes;
        for (int i = 0; i < size; i++)
        {
            if (filtered_label[i])
                continue;

            Point pt1 = before_nodes[i];
            MyNode node;
            node.points.push_back(Point(pt1));
            filtered_label[i] = true;

            int accNum = 0;
            do{
                accNum = 0;
                for (int j = 0; j < size; j++)
                {
                    if(j == i || filtered_label[j])
                        continue;
                    Point pt2 = before_nodes[j];
                    if (node.IsReachNode(pt2))
                    {
                        node.points.push_back(Point(pt2));
                        filtered_label[j] = true;
                        accNum++;
                    }
                }
            }while(accNum > 0);
            origin_nodes.push_back(MyNode(node));
        }

        size = origin_nodes.size();
        vector<int> true_nodes_label(size, 2);
        for(int i = 0; i < size; i++)
        {
            if(true_nodes_label[i] != 2)
                continue;
            
            MyNode* node_dst = &origin_nodes[i];
            true_nodes_label[i] = 1;
            int accNum = 0;
            do{
                accNum = 0;
                for(int j = 0; j < size; j++)
                {
                    if(true_nodes_label[j] != 2)
                        continue;
                    MyNode node_test = origin_nodes[j];
                    if(node_dst->distanceToNode(node_test) < 5.0)
                    {
                        node_dst->points.insert(node_dst->points.end(), node_test.points.begin(), node_test.points.end());
                        true_nodes_label[j] = 3;
                        accNum++;
                    }
                }
            }while(accNum > 0);
        }
        for(int i = 0; i < size; i++)
        {
            if(true_nodes_label[i] == 1)
                nodes.push_back(MyNode(origin_nodes[i]));
        }
        return nodes;
    }

    // extract branches and links
    void findChains(Mat skeleton, vector<MyNode> &nodes, vector<Point> &ends, vector<EndToNodeChain *> &end_to_node_chains, vector<NodeToNodeChain *> &node_to_node_chains)
    {
        Mat src = skeleton.clone();
        int height = src.rows, width = src.cols;
        // remove ends and nodes
        for(int i = 0; i < ends.size(); i++)
        {
            Point pt = ends[i];
            src.at<uchar>(pt.x, pt.y) = 0;
        }
        for(int i = 0; i < nodes.size(); i++)
        {
            MyNode node = nodes[i];
            node.drawNode(src, src, 0);
        }
        
        // 找到chains
        vector<vector<Point>*> chains;
        vector<Point> *one_chain = NULL;

        for(int x = 0; x < height; x++)
        {
            for(int y = 0; y < width; y++)
            {
                if(src.at<uchar>(x, y) == 0)
                    continue;
                
                int count = 0;
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0)
                            continue;

                        if (src.at<uchar>(x + i, y + j) > 200)
                        {
                            count++;
                        }
                    }
                }
                if(count != 1)
                    continue;
                
                one_chain = new vector<Point>();
                Point cur_pt = Point(x, y);
                one_chain->push_back(Point(cur_pt));
                src.at<uchar>(x, y) = 0;
                bool IsEndChain = false;

                while(!IsEndChain)
                {
                    bool IsFindNextPoint = false;
                    Point next_pt;
                    for (int i = -1; i <= 1; i++)
                    {
                        for (int j = -1; j <= 1; j++)
                        {
                            if (i == 0 && j == 0)
                                continue;

                            Point tmp_pt = Point(cur_pt.x + i, cur_pt.y + j);
                            if (src.at<uchar>(tmp_pt.x, tmp_pt.y) > 200)
                            {
                                next_pt = Point(tmp_pt);
                                src.at<uchar>(tmp_pt.x, tmp_pt.y) = 0;
                                IsFindNextPoint = true;
                                break;
                            }
                        }
                        if(IsFindNextPoint)
                            break;
                    }
                    if(IsFindNextPoint)
                    {
                        one_chain->push_back(Point(next_pt));
                        cur_pt = Point(next_pt);
                    }
                    else
                    {
                        IsEndChain = true;
                    }
                }
                chains.push_back(one_chain);
            }
        }

        
        vector<Point> true_ends;
        int true_end_index = 0;
        // 将chains分为end_to_end_chain和node_to_node_chain
        for(int i = 0; i < chains.size(); i++)
        {
            vector<Point> one_chain = *chains[i];
            Point end1 = one_chain[0], end2 = one_chain[one_chain.size()-1];
            int type1=-1, type2=-1;  // 0 for ends, 1 for nodes
            int index1 = -1, index2 = -1;

            for(int m = 0; m < ends.size(); m++)
            {
                if(type1 != -1 && type2 != -1)
                    break;

                if(type1 == -1)
                {
                    Point pt = ends[m];
                    if(DistanceBetweenPoints(end1, pt)<1.5)
                    {
                        type1 = 0;
                        index1 = m;
                        break;
                    }    
                }

                if(type2 == -1)
                {
                    Point pt = ends[m];
                    if(DistanceBetweenPoints(end2, pt)<1.5)
                    {
                        type2 = 0;
                        index2 = m;
                        break;
                    }    
                }
                
            }

            for(int m = 0; m < nodes.size(); m++)
            {
                if(type1 != -1 && type2 != -1)
                    break;

                if(type1 == -1)
                {
                    MyNode node = nodes[m];
                    if(node.IsReachNode(end1))
                    {
                        type1 = 1;
                        index1 = m;
                        continue;
                    }    
                }

                if(type2 == -1)
                {
                    MyNode node = nodes[m];
                    if(node.IsReachNode(end2))
                    {
                        type2 = 1;
                        index2 = m;
                    }    
                }
            }

            if(type1 == 0 && type2 == 1)
            {
                if(one_chain.size() >= 5){
                    EndToNodeChain *chain = new EndToNodeChain(end1);
                    chain->m_Node = nodes[index2];
                    chain->index_end = true_end_index++;
                    chain->index_node = index2;
                    chain->Chain = one_chain;
                    chain->length = chain->Length();
                    chain->IsReverse = false;
                    end_to_node_chains.push_back(chain);
                    true_ends.push_back(Point(end1));
                }
            }
            else if(type1 == 1 && type2 == 0)
            {
                if(one_chain.size() >= 5){
                    EndToNodeChain *chain = new EndToNodeChain(end2);
                    chain->m_Node = nodes[index1];
                    chain->index_end = true_end_index++;
                    chain->index_node = index1;
                    chain->Chain = one_chain;
                    chain->length = chain->Length();
                    chain->IsReverse = true;
                    end_to_node_chains.push_back(chain);
                    true_ends.push_back(Point(end2));
                }
            }
            else if(type1 == 1 && type2 == 1)
            {
                if(index1 == index2)
                {
                    cout<<"one node to node chain find same node" << endl;
                    continue;
                }
                NodeToNodeChain *chain = new NodeToNodeChain();
                chain->m_index1 = index1;
                chain->m_index2 = index2;
                chain->m_Node1 = nodes[index1];
                chain->m_Node2 = nodes[index2];
                chain->Chain = one_chain;
                chain->length = chain->Length();
                node_to_node_chains.push_back(chain);
            }
        }
        ends.clear();
        ends = vector<Point>(true_ends);
        for(auto chain:chains)
            delete chain;
    }

    // build connection matrix
    Mat getConnectionMat(vector<NodeToNodeChain *> &chains, vector<MyNode> nodes)
    {
        int size = nodes.size();
        Mat connect(size, size, CV_8U, Scalar(255));
        for (vector<NodeToNodeChain *>::iterator it = chains.begin(); it != chains.end(); it++)
        {
            NodeToNodeChain * chain = *it;
            int index1 = chain->m_index1;
            int index2 = chain->m_index2;
            connect.at<uchar>(index1, index2) = chain->length;
            connect.at<uchar>(index2, index1) = chain->length;
            connect.at<uchar>(index1, index1) = 0;
            connect.at<uchar>(index2, index2) = 0;
        }
        return connect;
    }

    // a sum up function to extract RAGVG
    Mat RAGVGExtraction(Mat skeleton, vector<Point> &ends_of_skeleton, vector<MyNode> &nodes_of_skeleton,
                        vector<EndToNodeChain *> &end_to_node_chain, vector<NodeToNodeChain *> &node_to_node_chain, Mat &connection_mat, 
                        std::vector<geometry_msgs::PointStamped> &output_nodes, int rows, int sliding_window_width, int interp_width)
    {
        nodes_of_skeleton.clear();
        ends_of_skeleton.clear();
        if(!end_to_node_chain.empty())
        {
            for(int i = 0; i<end_to_node_chain.size(); i++)
            {
                delete end_to_node_chain[i];
            }
            end_to_node_chain.clear();
        }
        
        if(!node_to_node_chain.empty())
        {
            for(int i = 0; i<node_to_node_chain.size(); i++)
            {
                delete node_to_node_chain[i];
            }
            node_to_node_chain.clear();
        }


        ends_of_skeleton = getEnds(skeleton);    // fixed param from experience
        nodes_of_skeleton = getNodes(skeleton);  // fixed param from experience
     
        findChains(skeleton, nodes_of_skeleton, ends_of_skeleton, end_to_node_chain, node_to_node_chain);
        connection_mat = getConnectionMat(node_to_node_chain, nodes_of_skeleton);


        Mat result = Mat(map.size(), CV_8UC3);
        cvtColor(map, result, COLOR_GRAY2RGB);

        // draw node_to_node chain
        // for (vector<NodeToNodeChain *>::iterator it = node_to_node_chain.begin(); it != node_to_node_chain.end(); it++)
        // {
        //     NodeToNodeChain * chain = (*it);
        //     for (auto k:chain->m_Node1.points)
        //     {
        //         cout<<"k popints x::"<<k.x<<"point y::"<<k.y;
        //     }

        //     chain->draw(result, result, Scalar(255, 0 , 0));
        // }

        // draw end_to_node chain
        for (vector<EndToNodeChain *>::iterator it = end_to_node_chain.begin(); it != end_to_node_chain.end(); it++)
        {
            EndToNodeChain * chain = (*it);
            // chain->draw(result, result, Scalar(0, 255 , 0));
        }

        // draw nodes
        for (vector<MyNode>::iterator it = nodes_of_skeleton.begin(); it != nodes_of_skeleton.end(); it++)
        {
            Point node = it->centerPoint();
            // cout<<"nodes of skeleton::"<<nodes_of_skeleton.at(it).point.x<<" "<<nodes_of_skeleton.at(it).point.y<<"\n";
            circle(result, Point(node.y, node.x), 3, Scalar(0, 255 , 255), 1, 8);
        }

        // draw circles at ends
        for (vector<Point>::iterator it = ends_of_skeleton.begin(); it != ends_of_skeleton.end(); it++)
        {
            Point end = *it;
            // cout<<"nodes of skeleton:::"<<ends_of_skeleton.at(it).point.x<<" "<<nodes_of_skeleton.at(it).point.y<<"\n";
            circle(result, Point(end.y, end.x), 3, Scalar(255, 0 , 255), 1, 8);
        }
        // for (auto e : ends_of_skeleton)
        // {
        //     cout<<"nodes of end of skeleton::"<<e.x<<" "<<e.y<<"\n";
        // }
        //  for (auto n : nodes_of_skeleton)
        // {
        //     // Point node = n;
        //     cout<<"nodes of end of skeleton::"<<n.x<<" "<<n.y<<"\n";
        // }
        int count = 0;
        geometry_msgs::PointStamped p;
        std::cout<<"rows value::"<<rows<<"\n";
        for (vector<NodeToNodeChain *>::iterator it = node_to_node_chain.begin(); it != node_to_node_chain.end(); it++)
        {
            NodeToNodeChain * chain = (*it);
                
            std::cout<<"inside node to node draw chain \n";
            for (auto k:chain->m_Node1.points)
            {
                std::cout<<"inside for loop \n";
                cout<<"k popints x::"<<k.x<<"point y::"<<k.y<<"\n";
                p.point.x = (int)k.y;
                p.point.y = rows - (int)k.x;
                int flag = 0;
                for (auto i:output_nodes)
                {
                    if ((abs(i.point.x-p.point.x)<=sliding_window_width) &&(abs(i.point.y-p.point.y)<=sliding_window_width))
                    {    
                        // cout<<"flag 1 condition reached \n";
                        // cout<<"values::::output_nodes::"<<i.point.x<<" "<<i.point.y<<"\n";
                        // cout<<"nodes of skeleton::"<<p.point.x<<" "<<p.point.y<<"\n";
                        flag = 1;
                    }
                    
                }
                if (flag ==0)
                {    
                    if (p.point.x >= 640) {
                        continue;
                    }
                    if(((p.point.x == 66) && (p.point.y == 286)) || ((p.point.x == 473) && (p.point.y == 139)) || ((p.point.x==445) && (p.point.y==143)))
                    {
                        cout<<"inside removal condition!!!!! \n";
                        continue;
                    }
                    output_nodes.push_back(p);
                }
                    // output_nodes.push_back(p);
            }
            chain->draw(result, result, Scalar(255, 0 , 0));
        }
        std::cout<<"exited node to node for loop:::"<<output_nodes.size()<<"\n";
        

        return result;
    }
    
};
