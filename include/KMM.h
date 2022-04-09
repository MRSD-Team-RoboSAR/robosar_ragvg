#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
using namespace cv;
using namespace std;

#pragma region ParallelKMM
// ********* parallel KMM ****************
int A4[24] = { 3, 6, 12, 24, 48, 96, 192, 129, 7, 14, 28, 56, 112, 224,
193, 131, 15, 30, 60, 120, 240, 225, 195, 135 };
int EDGE[239] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
81, 82, 83, 84, 86, 88, 89, 90, 91, 92, 94, 96, 97, 98, 99, 100, 101, 102, 103,
104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 118, 120, 121,
122, 123, 124, 126, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155,
156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171,
172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187,
188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203,
204, 205, 206, 207, 208, 209, 210, 211, 212, 214, 216, 217, 218, 219, 220, 222,
224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
240, 241, 242, 243, 244, 246, 248, 249, 250, 251, 252, 254 };
int CORNER[15] = { 253, 247, 223, 127, 245, 221, 125, 215, 119, 95, 213, 87, 93, 117, 85 };
int DLA[120] = { 3, 5, 7, 12, 13, 14, 15, 20,
21, 22, 23, 28, 29, 30, 31, 48,
52, 53, 54, 55, 56, 60, 61, 62,
63, 65, 67, 69, 71, 77, 79, 80,
81, 83, 84, 85, 86, 87, 88, 89,
91, 92, 93, 94, 95, 97, 99, 101,
103, 109, 111, 112, 113, 115, 116, 117,
118, 119, 120, 121, 123, 124, 125, 126,
127, 131, 133, 135, 141, 143, 149, 151,
157, 159, 181, 183, 189, 191, 192, 193,
195, 197, 199, 205, 207, 208, 209, 211,
212, 213, 214, 215, 216, 217, 219, 220,
221, 222, 223, 224, 225, 227, 229, 231,
237, 239, 240, 241, 243, 244, 245, 246,
247, 248, 249, 251, 252, 253, 254, 255 };

class FindBoundaryParallel :public ParallelLoopBody
{
public:
	Mat &image;
	int width = 0, height = 0;
	int neighbor[3][3];
	//vector<pair<Point, int>> result;
	FindBoundaryParallel(Mat &data) : image(data)
	{
		width = data.cols;
		height = data.rows;
		neighbor[0][0] = 128;
		neighbor[0][1] = 1;
		neighbor[0][2] = 2;
		neighbor[1][0] = 64;
		neighbor[1][1] = 0;
		neighbor[1][2] = 4;
		neighbor[2][0] = 32;
		neighbor[2][1] = 16;
		neighbor[2][2] = 8;
	}

	int calWeight(Mat &src, Point pt) const
	{
		int weight = 0;
		for (int i = -1; i < 2; i++)
		{
			uchar* irow = src.ptr(pt.x + i);
			for (int j = -1; j < 2; j++)
			{
				if (irow[pt.y + j] != 0)
					weight += neighbor[i + 1][j + 1];
			}
		}
		return weight;
	}

	bool isInArray(int weight, int arr[], int size) const
	{
		for (int i = 0; i < size; i++)
		{
			if (arr[i] == weight)
				return true;
		}
		return false;
	}

	virtual void operator ()(const Range& range) const
	{
		for (int r = range.start; r < range.end; r++)
		{
			int x = r / width;
			int y = r % width;

			uchar* irow = image.ptr(x);
			if (irow[y] == 0)
				continue;
			else
			{
				int weight = calWeight(image, Point(x, y));
				if (weight == 255)
					continue;
				else
					irow[y] = weight;
			}
		}
	}

	FindBoundaryParallel& operator=(const FindBoundaryParallel &) {
		return *this;
	}
};

class EDGEParallel :public ParallelLoopBody
{
public:
	Mat &image;
	int width = 0, height = 0;
	int neighbor[3][3];
	vector<pair<Point, int>> boundary;
	EDGEParallel(Mat &data, vector<pair<Point, int>> &boundary) : image(data), boundary(boundary)
	{
		width = data.cols;
		height = data.rows;
		neighbor[0][0] = 128;
		neighbor[0][1] = 1;
		neighbor[0][2] = 2;
		neighbor[1][0] = 64;
		neighbor[1][1] = 0;
		neighbor[1][2] = 4;
		neighbor[2][0] = 32;
		neighbor[2][1] = 16;
		neighbor[2][2] = 8;
	}

	int calWeight(Mat &src, Point pt) const
	{
		int weight = 0;
		for (int i = -1; i < 2; i++)
		{
			uchar* irow = src.ptr(pt.x + i);
			for (int j = -1; j < 2; j++)
			{
				if (irow[pt.y + j] != 0)
					weight += neighbor[i + 1][j + 1];
			}
		}
		return weight;
	}

	bool isInArray(int weight, int arr[], int size) const
	{
		for (int i = 0; i < size; i++)
		{
			if (arr[i] == weight)
				return true;
		}
		return false;
	}

	virtual void operator ()(const Range& range) const
	{
		for (int r = range.start; r < range.end; r++)
		{
			Point pt = boundary[r].first;
			int weight = boundary[r].second;
			int y = pt.y;   // y
			int x = pt.x;   // x

			uchar* irow = image.ptr(x);
			if (irow[y] == 0)
				continue;
			else
			{
				int weight = calWeight(image, Point(x, y));
				if (weight == 255)
					continue;
				else if (!isInArray(weight, CORNER, 15))
					irow[y] = 2;
				//result.push_back(make_pair(Point(x, y), weight));
			}
		}
	}

	EDGEParallel& operator=(const EDGEParallel &) {
		return *this;
	}
};

class CORNERParallel :public ParallelLoopBody
{
public:
	Mat &image;
	int width = 0, height = 0;
	int neighbor[3][3];
	vector<pair<Point, int>> boundary;
	CORNERParallel(Mat &data, vector<pair<Point, int>> &boundary) : image(data), boundary(boundary)
	{
		width = data.cols;
		height = data.rows;
		neighbor[0][0] = 128;
		neighbor[0][1] = 1;
		neighbor[0][2] = 2;
		neighbor[1][0] = 64;
		neighbor[1][1] = 0;
		neighbor[1][2] = 4;
		neighbor[2][0] = 32;
		neighbor[2][1] = 16;
		neighbor[2][2] = 8;
	}

	int calWeight(Mat &src, Point pt) const
	{
		int weight = 0;
		for (int i = -1; i < 2; i++)
		{
			uchar* irow = src.ptr(pt.x + i);
			for (int j = -1; j < 2; j++)
			{
				if (irow[pt.y + j] != 0)
					weight += neighbor[i + 1][j + 1];
			}
		}
		return weight;
	}

	bool isInArray(int weight, int arr[], int size) const
	{
		for (int i = 0; i < size; i++)
		{
			if (arr[i] == weight)
				return true;
		}
		return false;
	}

	virtual void operator ()(const Range& range) const
	{
		for (int r = range.start; r < range.end; r++)
		{
			Point pt = boundary[r].first;
			int weight = boundary[r].second;
			int y = pt.y;   // y
			int x = pt.x;   // x

			uchar* irow = image.ptr(x);
			if (irow[y] == 0)
				continue;
			else
			{
				int weight = calWeight(image, Point(x, y));
				if (weight == 255)
					continue;
				else if (isInArray(weight, CORNER, 15))
					irow[y] = 3;
				//result.push_back(make_pair(Point(x, y), weight));
			}
		}
	}

	CORNERParallel& operator=(const CORNERParallel &) {
		return *this;
	}
};

class STICKINGParallel :public ParallelLoopBody
{
public:
	Mat &image;
	int width = 0, height = 0;
	int neighbor[3][3];
	vector<pair<Point, int>> boundary;
	bool *ismodified;
	STICKINGParallel(Mat &data, vector<pair<Point, int>> boundary, bool *ismodified) : image(data), boundary(boundary), ismodified(ismodified)
	{
		width = data.cols;
		height = data.rows;
		neighbor[0][0] = 128;
		neighbor[0][1] = 1;
		neighbor[0][2] = 2;
		neighbor[1][0] = 64;
		neighbor[1][1] = 0;
		neighbor[1][2] = 4;
		neighbor[2][0] = 32;
		neighbor[2][1] = 16;
		neighbor[2][2] = 8;
	}

	int calWeight(Mat &src, Point pt) const
	{
		int weight = 0;
		for (int i = -1; i < 2; i++)
		{
			uchar* irow = src.ptr(pt.x + i);
			for (int j = -1; j < 2; j++)
			{
				if (irow[pt.y + j] != 0)
					weight += neighbor[i + 1][j + 1];
			}
		}
		return weight;
	}

	bool isInArray(int weight, int arr[], int size) const
	{
		for (int i = 0; i < size; i++)
		{
			if (arr[i] == weight)
				return true;
		}
		return false;
	}

	virtual void operator ()(const Range& range) const
	{
		for (int r = range.start; r < range.end; r++)
		{
			Point pt = boundary[r].first;
			int weight = boundary[r].second;
			int y = pt.y;   // y
			int x = pt.x;   // x

			uchar* irow = image.ptr(x);
			if (irow[y] == 0)
				continue;
			else
			{
				int weight = calWeight(image, Point(x, y));
				if (weight == 255)
					continue;
				else if (isInArray(weight, A4, 24))
				{
					irow[y] = 4;
					*ismodified = true;
				}
			}
		}
	}

	STICKINGParallel& operator=(const STICKINGParallel &) {
		return *this;
	}
};

class KMM
{
public:
	Mat image;
	int neighbor[3][3];
	int width = 0, height = 0;
	KMM(Mat &data)
	{
		image = Mat::zeros(data.size(), CV_8U);
		image = data.clone();
		//{128, 1, 2, 64, 0, 4, 32, 16, 8}
		neighbor[0][0] = 128;
		neighbor[0][1] = 1;
		neighbor[0][2] = 2;
		neighbor[1][0] = 64;
		neighbor[1][1] = 0;
		neighbor[1][2] = 4;
		neighbor[2][0] = 32;
		neighbor[2][1] = 16;
		neighbor[2][2] = 8;
		width = data.cols;
		height = data.rows;
	}

	int calWeight(Mat &src, Point pt)
	{
		int weight = 0;
		for (int i = -1; i < 2; i++)
		{
			uchar* irow = src.ptr(pt.x + i);
			for (int j = -1; j < 2; j++)
			{
				if (irow[pt.y + j] != 0)
					weight += neighbor[i + 1][j + 1];
			}
		}
		return weight;
	}

	vector<pair<Point, int>> findBoundary(Mat src)
	{
		vector<pair<Point, int>> result;
		for (int i = 0; i < height - 1; i++)
		{
			uchar* irow = src.ptr(i);
			for (int j = 0; j < width - 1; j++)
			{
				if (irow[j] == 0)
					continue;
				else
				{
					int weight = calWeight(src, Point(i, j));
					if (weight == 255)
						continue;
					else
						result.push_back(make_pair(Point(i, j), weight));
				}
			}
		}
		return result;
	}

	bool isInArray(int weight, int arr[], int size)
	{
		for (int i = 0; i < size; i++)
		{
			if (arr[i] == weight)
				return true;
		}
		return false;
	}

	bool change(Mat &src, vector<pair<Point, int>> boundary, int arr[], int size, int dest, int mode)
	{
		bool ischange = false;

		for (vector<pair<Point, int>>::iterator it = boundary.begin(); it != boundary.end(); it++)
		{
			int weight = it->second;
			Point pt = it->first;
			if (mode)
			{
				if (isInArray(weight, arr, size))
				{
					uchar* irow = src.ptr(pt.x);
					irow[pt.y] = dest;
					ischange = true;
				}
			}
			else
			{
				if (!isInArray(weight, arr, size))
				{
					uchar* irow = src.ptr(pt.x);
					irow[pt.y] = dest;
					ischange = true;
				}
			}
		}
		return ischange;
	}

	Mat execute()
	{
		int A4[24] = { 3, 6, 12, 24, 48, 96, 192, 129, 7, 14, 28, 56, 112, 224,
			193, 131, 15, 30, 60, 120, 240, 225, 195, 135 };
		int EDGE[239] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
			21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
			41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
			61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
			81, 82, 83, 84, 86, 88, 89, 90, 91, 92, 94, 96, 97, 98, 99, 100, 101, 102, 103,
			104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 118, 120, 121,
			122, 123, 124, 126, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
			140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155,
			156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171,
			172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187,
			188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203,
			204, 205, 206, 207, 208, 209, 210, 211, 212, 214, 216, 217, 218, 219, 220, 222,
			224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
			240, 241, 242, 243, 244, 246, 248, 249, 250, 251, 252, 254 };
		int CORNER[15] = { 253, 247, 223, 127, 245, 221, 125, 215, 119, 95, 213, 87, 93, 117, 85 };
		int DLA[120] = { 3, 5, 7, 12, 13, 14, 15, 20,
			21, 22, 23, 28, 29, 30, 31, 48,
			52, 53, 54, 55, 56, 60, 61, 62,
			63, 65, 67, 69, 71, 77, 79, 80,
			81, 83, 84, 85, 86, 87, 88, 89,
			91, 92, 93, 94, 95, 97, 99, 101,
			103, 109, 111, 112, 113, 115, 116, 117,
			118, 119, 120, 121, 123, 124, 125, 126,
			127, 131, 133, 135, 141, 143, 149, 151,
			157, 159, 181, 183, 189, 191, 192, 193,
			195, 197, 199, 205, 207, 208, 209, 211,
			212, 213, 214, 215, 216, 217, 219, 220,
			221, 222, 223, 224, 225, 227, 229, 231,
			237, 239, 240, 241, 243, 244, 245, 246,
			247, 248, 249, 251, 252, 253, 254, 255 };


		bool ismodified = false;
		Mat tmp = image.clone();
		tmp /= 255;
		int circlenum = 0;
		int numcore = 8;
		do{
			clock_t begintime, endtime;
			//begintime = clock();
			circlenum++;
			ismodified = false;
			int count = 0;
			vector<pair<Point, int>> boundary;
			FindBoundaryParallel FB(tmp);
			parallel_for_(Range(0, tmp.cols * tmp.rows), FB, numcore);

			for (int i = 0; i < height - 1; i++)
			{
				uchar* irow = tmp.ptr(i);
				for (int j = 0; j < width - 1; j++)
				{
					uchar p = irow[j];
					if (p>1)
						boundary.push_back(make_pair(Point(i, j), p));
				}
			}
			//printf("boundary size:%d   ", boundary.size());
			EDGEParallel EP(tmp, boundary);
			parallel_for_(Range(0, boundary.size()), EP, numcore);
			CORNERParallel CP(tmp, boundary);
			parallel_for_(Range(0, boundary.size()), CP, numcore);
			STICKINGParallel SP(tmp, boundary, &ismodified);
			parallel_for_(Range(0, boundary.size()), SP, numcore);
			//change(tmp, boundary, CORNER, 15, 2, 0);
			//change(tmp, boundary, CORNER, 15, 3, 1);
			//ismodified = change(tmp, boundary, A4, 24, 4, 1);

			for (int i = 0; i < height - 1; i++)
			{
				uchar* irow = tmp.ptr(i);
				for (int j = 0; j < width - 1; j++)
				{
					if (irow[j] == 4)
					{
						irow[j] = 0;
						count++;
					}
				}
			}

			for (int i = 0; i < height - 1; i++)
			{
				uchar* irow = tmp.ptr(i);
				for (int j = 0; j < width - 1; j++)
				{
					if (irow[j] == 2)
					{
						if (isInArray(calWeight(tmp, Point(i, j)), DLA, 120))
						{
							irow[j] = 0;
							ismodified = true;
							count++;
						}
						else
							irow[j] = 1;
					}
				}
			}

			for (int i = 0; i < height - 1; i++)
			{
				uchar* irow = tmp.ptr(i);
				for (int j = 0; j < width - 1; j++)
				{
					if (irow[j] == 3)
					{
						if (isInArray(calWeight(tmp, Point(i, j)), DLA, 120))
						{
							irow[j] = 0;
							ismodified = true;
							count++;
						}
						else
							irow[j] = 1;
					}
				}
			}
			//endtime = clock();
			//double dur = (double)(endtime - begintime);
			//printf("%d KMM Time:%f\n", circlenum, dur);
		} while (ismodified);
		return tmp * 255;
	}
};

#pragma endregion ParallelKMM