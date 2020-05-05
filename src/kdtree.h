/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include<math.h>

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
		
	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL){
			
			*node = new Node(point, id);		
		}
		else{
			int cd = depth %2;
			
			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);		
		}
	}	


	void insert(std::vector<float> point, int id)
	{		
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	void searchhelper(Node** node, std::vector<float> target, int depth, float distanceTol, std::vector<int>& ids){

		if (*node != NULL){
			
			if ((abs((*node)->point[0] - target[0]) <= distanceTol)&&(abs((*node)->point[1] - target[1]) <= distanceTol)){
				if (sqrt(pow(((*node)->point[0] - target[0]),2)+pow(((*node)->point[1] - target[1]),2)) <= distanceTol){
					ids.push_back((*node)->id);
				}	
			}
		
			int cd = depth%2;		
			float max = target[cd] + distanceTol;
			float min = target[cd] - distanceTol;
		
			if ((min < (*node)->point[cd])){
				searchhelper(&((*node)->left), target, depth+1, distanceTol, ids);
			}
			if(max >  (*node)->point[cd]){
			
				searchhelper(&((*node)->right), target, depth+1, distanceTol, ids);
				
			}  

			
		}

	}
	
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchhelper(&root, target, 0, distanceTol, ids);
		return ids;
	}
	

};




