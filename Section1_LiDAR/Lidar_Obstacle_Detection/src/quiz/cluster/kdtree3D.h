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

struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	~KdTree3D()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// Step 1: If root is null, create a new node and set it as root
		
		if (root == NULL)
		{
			root = new Node(point, id);
			return;
		}
		// Step 2: Otherwise, call the recursive insert function
		insertHelper(root, point, id, 0);
		

	}
	void insertHelper(Node* node, std::vector<float> point, int id, uint depth)
	{
		if (node == NULL)
			return;
		
		// Step 3: Compare the point with the current node's point based on depth
		uint cd = depth % 3; // 0 for x-axis, 1 for y-axis, 2 for z-axis
		if (point[cd] < node->point[cd])
		{
			// Step 4: If the point is less, go left
			if (node->left == NULL)
			{
				node->left = new Node(point, id);
			}
			else
			{
				insertHelper(node->left, point, id, depth + 1);
			}
		}
		else
		{
			// Step 5: If the point is greater or equal, go right
			if (node->right == NULL)
			{
				node->right = new Node(point, id);
			}
			else
			{
				insertHelper(node->right, point, id, depth + 1);
			}
		}
		// Step 6: If the point is equal, we can choose to ignore it
		// or handle it as per requirement (e.g., update the id or store multiple ids)
		// For simplicity, we will ignore it here.
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}
	
	void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, uint depth)
	{
		if (node == NULL)
		{
			// std::cout << "Null Node" << std::endl;
			return;
		}
		

		// Step 1: Check if the current node is within the distance tolerance
		if (node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol &&
			node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol &&
			node->point[2] >= target[2] - distanceTol && node->point[2] <= target[2] + distanceTol)
		{
			float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));
			if (distance <= distanceTol)
			{
				ids.push_back(node->id);
				// std::cout << "Nearby Node" << std::endl;
			}
		}

		// Step 2: Determine which side of the tree to search based on depth
		uint cd = depth % 3; // 0 for x-axis, 1 for y-axis, 2 for z-axis
		if (target[cd] - distanceTol < node->point[cd])
		{
			searchHelper(node->left, target, distanceTol, ids, depth + 1);
		}
		if (target[cd] + distanceTol > node->point[cd])
		{
			searchHelper(node->right, target, distanceTol, ids, depth + 1);
		}
	}

};




