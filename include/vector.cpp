	MyNode(const MyNode &node)
	{
		int size = node.points.size();
        points = vector<Point>(node.points);
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
