/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

typedef unsigned int unit;

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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHandler(Node** node, unit depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
			*node = new Node(point, id);
		else
		{
			unit cd = depth % 2;
			
			if(point[cd] < (*node)->point[cd])
			{
			   insertHandler(&((*node)->left), depth + 1, point, id);
			}
			else
			{				
			   insertHandler(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHandler(&root, 0, point, id);	
	}

	void searchHandler(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1] + distanceTol)))
			{
				float delta_x = node->point[0] - target[0];
				float delta_y = node->point[1] - target[1];
				float distance = std::sqrt( (delta_x * delta_x) + (delta_y * delta_y) );
				if(distance < distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if((target[depth%2]-distanceTol) < node->point[depth%2])
			{
				searchHandler(target, node->left, depth + 1, distanceTol, ids);
			}
			if((target[depth%2]+distanceTol) > node->point[depth % 2])
			{
				searchHandler(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHandler(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




