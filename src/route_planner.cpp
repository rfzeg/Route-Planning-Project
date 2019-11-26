#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
  // Store the nodes you find in the RoutePlanner's start_node and end_node attributes
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// When the class methods are defined outside the class, the scope resolution operator ::
// indicates which class the method belongs to
float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
  // Use the distance to the end_node for the h value.
  // Node objects have a distance method to determine the distance to another node.
  // Use -> to simultaneously dereference the pointer node (equivalent to writing *node)
  // and access the distance() method (equivalent to using the . (dot) notation
  return node->distance(*end_node);
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector
  current_node->FindNeighbors();
  // For each node in current_node.neighbors, set the parent, the h_value, the g_value
  for (auto n : current_node->neighbors){
    n->parent = current_node;
    // Use CalculateHValue below to implement the h-Value calculation
    n->h_value = CalculateHValue(n);
    // Calculate g, the cost for each move
    n->g_value = current_node->g_value + current_node->distance(*n);
    // Add the neighbor to open_list and set the node's visited attribute to true
    open_list.push_back(n);
    n->visited = true;
  }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
std::sort(open_list.begin(),open_list.end(),
  // Lambda expression begins, it allows to write an inline anonymous functor
  [](const auto &first_n, const auto &second_n)
  {
    return first_n->h_value + first_n->g_value < second_n->h_value + second_n->g_value;
  }// end of lambda expression
  );
    // Create a pointer to the node in the list with the lowest sum
    RouteModel::Node *lowest_F = *(open_list.begin());
    // Remove that node from the open_list
    open_list.erase(open_list.begin());
    // Return the pointer
    return lowest_F;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  // Iteratively follow the chain of parents of nodes until the starting node is found
  while(current_node->parent != nullptr){
	  path_found.push_back(*current_node);
    // For each node, add the distance from the node to its parent to the distance variable
		distance += current_node->distance(*(current_node->parent));
		current_node = current_node->parent;
  }
	path_found.push_back(*current_node);
  // Rearrange to the correct order: the start node should be the first element
  // of the vector, the end node should be the last element
  reverse(path_found.begin(), path_found.end());
  // Multiply the distance by the scale of the map to get meters
  distance *= m_Model.MetricScale();
  return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}