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
};

struct KdTree
{
	Node* root;
    int numDimensions;

	KdTree()
    : root(NULL), numDimensions(2)
	{}

    bool toLeft(int level, std::vector<float> point, std::vector<float> parent_point)
    {
        int split_dim = level%numDimensions;
        return ( point[split_dim] <= parent_point[split_dim] );
    }

    bool toLeft(int level, std::vector<float> point, std::vector<float> parent_point, float &distance_split_dim, float&distance_max)
    {
        int split_dim = level%numDimensions;
        distance_split_dim = abs(point[split_dim] - parent_point[split_dim]);
        distance_max = 0;
        for (int i=0; i<numDimensions; i++)
        {
            float d = abs(point[i] - parent_point[i]);
            if (d > distance_max)
                distance_max = d;
        }
        return ( point[split_dim] <= parent_point[split_dim] );
    }

	void insert(std::vector<float> point, int id)
	{
        Node *new_node = new Node(point, id);
        insertNode(new_node, root, 0);
	}

    void insertNode(Node *&node, Node *&parent, int depth)
    {
        if(parent == NULL)
        {
            parent = node;
        }
        else
        {
            if(toLeft(depth, node->point, parent->point))
            {
                depth++;
                insertNode(node, parent->left, depth);
            }
            else
            {
                depth++;
                insertNode(node, parent->right, depth);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchSubtree(ids, root, target, distanceTol, 0);
        return ids;
    }

    void searchSubtree(std::vector<int> &ids, Node *node, std::vector<float> &target, float distanceTol, int depth)
    {
        if (node == NULL)
            return;
        float distance_split_dim = 0;
        float distance_max = 0;
        bool pointToLeft = toLeft(depth, target, node->point, distance_split_dim, distance_max);
        if (distance_max < distanceTol)
        {
            ids.push_back(node->id);
        }
        if (pointToLeft)
        {
            depth++;
            searchSubtree(ids, node->left, target, distanceTol, depth);
            if (distance_split_dim < distanceTol)
                searchSubtree(ids, node->right, target, distanceTol, depth);
        }
        else
        {
            depth++;
            searchSubtree(ids, node->right, target, distanceTol, depth);
            if (distance_split_dim < distanceTol)
                searchSubtree(ids, node->left, target, distanceTol, depth);
        }
    }

};




