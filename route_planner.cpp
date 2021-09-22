#include "route_planner.h"
#include "route_model.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    //You can use the distance to the end_node for the h value.
    float h = node->distance(*end_node); //-> access distance method for pointer node (usually . when not a pointer)
    return h;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
//Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
current_node->FindNeighbors();
//For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    //Use CalculateHValue below to implement the h-Value calculation.
    neighbor->h_value = CalculateHValue(neighbor);
    //new g value
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    // For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}


RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list according to the sum of the h value and g value.
  sort(open_list.begin(), open_list.end(), [](const auto &n1, const auto &n2){
    return (n1->g_value + n1->h_value) < (n2->g_value + n2->h_value);
  });
  // Create a pointer to the node in the list with the lowest sum.
  RouteModel::Node *least_f_value = open_list.front();
  // Remove that node from the open_list.
  open_list.erase(open_list.begin());
  // return pointer
  return least_f_value;

}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found ={};

    //going back up the chain of parents until the starting node (no parents)
    while (current_node->parent != nullptr) {
      //add node to the path by the end to get final vector in order from start to finish
      path_found.push_back(*current_node);
      //add the distance from the node to its parent to the distance variable.
      distance += current_node->distance(*(current_node->parent));
      //set current node to its parent
      current_node = current_node->parent;
    }
    // add last node
    path_found.push_back(*current_node); 

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {

  RouteModel::Node *current_node = nullptr;

  //Mark the starting node as visited and adding it to the node list to initialize the loop
  start_node->visited = true;
  open_list.push_back(start_node);

  // start the loop while until there is no node to explore
  while (open_list.size() != 0) {
    //assign the pointer to the node with lowest f value to the pointer current_node. Also remove the low-f-value node from the open list
    current_node = NextNode();
    //we reached the end node, return the path
    if (current_node->distance(*end_node) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
    }
    // keep looking for a path if we have not reach the end node
    AddNeighbors(current_node);
  }
}