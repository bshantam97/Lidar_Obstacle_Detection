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

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void _insert (Node *& node, std::vector<float> points, uint depth, int id) {
		if (node == nullptr) {
			node = new Node(points, id);
		} else {
			// Now the logic behind this is 0 is for x axis and 1 is for y axis
			// Even depths are for x alignment and odd depths are for y alignment
			uint currentDim = depth % 3;
			if (points[currentDim] < (node->point[currentDim])) {
				_insert(node->left, points, depth+1,id);
			} else {
				_insert(node->right, points, depth+1, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		_insert(root, point, 0, id); 
	}

	// For the search function we will need a helper function.
	// Passing ids as a reference. Will be available outside the function too when modified.
	// The node has an associated id with it 
	void _search(Node * node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids) {
		if (node == nullptr) {
			return;
		}
		if (node != nullptr) {
			// First we check whether it is in the box/cube region or not 
			if (node->point[0] >= (target[0] - distanceTol) && 
			node->point[0] <= (target[0] + distanceTol) && 
			node->point[1] >= (target[1] - distanceTol) && 
			node->point[1] <= (target[1] + distanceTol) &&
			node->point[2] >= (target[2] - distanceTol) &&
			node->point[2] <= (target[2] + distanceTol)) {
				// If we have determined that the point is in the box 
				// we need to determine whether it is in the circle or not
				auto distance = std::sqrt(std::pow((node->point[0]-target[0]),2) + std::pow((node->point[1]-target[1]),2));
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}
			// The next if statement will be very similar to the insert function
			// Below We calculated the current dimension. 0 = x and 1 = y and 2 = z
			// The logic is that at depth 0 we get an x axis divison, depth 1 gives y axis division and depth 2 gives z axis division
			uint current_dim = depth % 3;
			// What we need to do now is check across the boundary whether the boundary point is less than
			// Basically comapare the edges
			if ((target[current_dim]-distanceTol) < node->point[current_dim])
				_search(node->left, target, distanceTol, depth+1, ids);
			if ((target[current_dim] + distanceTol) > node->point[current_dim])
				_search(node->right, target, distanceTol, depth+1, ids);
		}
	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		_search(root, target, distanceTol, 0, ids);
		return ids;
	}
};