/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;
	int idx;
	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// either use return or use a double pointer/pointer by reference. 
	// using the latter methods, the changes are directly reflected in the calling function
	void insertRec(Node** root, std::vector<float> point, int depth, int id)
	{
		if (*root == NULL)
		{
			*root = new Node(point, id);
		}
		else
		{
			idx = depth % 2;
			if (point[idx] < ((*root)->point[idx]))
				//(*root)->left = insertRec(&((*root)->left), point, depth + 1, id);
				insertRec(&((*root)->left), point, depth + 1, id);
			else
				//(*root)->right = insertRec(&((*root)->right), point, depth + 1, id);
				insertRec(&((*root)->right), point, depth + 1, id);
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRec(&root, point, 0, id);
	}

	// return a list of point ids in the tree that are within distance of target
	bool distCompare(std::vector<float> point1, std::vector<float> point2, float distTol)
	{
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
		std::cout << x << std::endl;
		if (sqrt((x * x + y * y)) <= distTol)
		{	
			//std::cout << sqrt((x * x + y * y)) << std::endl;
			return true;
		}
		return false;
	}

	void searchRec(std::vector<float> target, Node* root, int depth, float distTol, std::vector<int>& ids)
	{
		if (root != NULL)
		{	
			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))) {
				if (distCompare(target, root->point, distTol))
				{
					ids.push_back(root->id);
				}
			}

			idx = depth % 2;
			if ((target[idx] - distTol) < root->point[idx])
				searchRec(target, root->left, depth + 1, distTol, ids);

			// if target X + dist tol is still less than the point's X, then don't go down the right side of the tree
			if ((target[idx] + distTol) > root->point[idx])
				searchRec(target, root->right, depth + 1, distTol, ids);
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRec(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




