#include "./Planner/planning.h"
#include <algorithm>


double calculate_distance(Node node_A, Node node_B){
    return sqrt((node_A.x-node_B.x)*(node_A.x-node_B.x) + (node_A.y-node_B.y)*(node_A.y-node_B.y));
}

void Planner::planPath(){
    
    /* 0: Initialization */
    int n_nodes = _nodelist.size();
    Node node_goal = _nodelist[_finish_nodeID];
    for (int nodeID = 0; nodeID < n_nodes; nodeID++){
        _nodelist[nodeID].g = INFINITY; // cost-to-come from the start to this node
        _nodelist[nodeID].h = calculate_distance(_nodelist[nodeID], node_goal); // heuristic cost-to-go from this node to the goal
        _nodelist[nodeID].f = _nodelist[nodeID].g + _nodelist[nodeID].h; // cost-to-come + cost-to-go
        _nodelist[nodeID].parent_node_ID = NODE_ID_NOT_SET;  // ID of the node from which node you arrived at this node with the lowest cost-to-come
    }
    std::list<int> open_nodes = {_entrance_nodeID};
    std::list<int> closed_nodes = {};
    _nodelist[_entrance_nodeID].g = 0.0;
    _nodelist[_entrance_nodeID].f = _nodelist[_entrance_nodeID].g + _nodelist[_entrance_nodeID].h;
    bool goal_reached = false;
    
    /* 1: Plan path using the A* algorithm with:
     * - cost-to-come g: distance covered from the start to the node
     * - heuristic cost-to-go h: straight line distance from the node to the goal */
    while (!goal_reached && !open_nodes.empty())
    {
        /* Expand open node with lowest f (except when it's already the goal):
         * - explore its connected nodes and if the cost-to-come to such a node via this note is lower, update it. Open them if they were not open yet.
         * - close this node */

        // Find the node number from the ones that are open with minimum f
        int nodeID_minimum_f = NODE_ID_NOT_SET;
        /*-- Exercise 1/3: Find the index of the node with minimum f and store it in the variable nodeID_minimum_f --*/
        // Put your code here
        int min_f =INFINITY;
        for (int i : open_nodes) {
            if(_nodelist[i].f< min_f) 
            {
                min_f = _nodelist[i].f; //open_nodes[i]  open_nodes
                nodeID_minimum_f =i;
            }
        }
        /* End Exercise 1/3 */

        // The goal is reached (and via the optimal path) when the goal is the open node with minimum f
        if (nodeID_minimum_f == _finish_nodeID) {
            goal_reached = true;
        } else if (nodeID_minimum_f == NODE_ID_NOT_SET) {
            break;
        } else {
            int current_nodeID = nodeID_minimum_f;
            /*-- Exercise 2/3: Explore the nodes connected to the new current node if they were not closed yet:
             *                 - Add them to the list of open nodes when they were not open yet.
             *                 - If the cost-to-come to such a node via the current node is lower than it was via another node, update it:
             *                     - update g (and f accordingly)
             *                     - update its parent node (should now become equal to the current node) */
           // Put your code here
            for(int next_visit_idx : _connections[current_nodeID])
            {  

                bool found_closed = (std::find(closed_nodes.begin(), closed_nodes.end(), next_visit_idx) != closed_nodes.end()); // true if found
                bool found_opened = (std::find(open_nodes.begin(), open_nodes.end(), next_visit_idx) != open_nodes.end());

                // calculate the current distance
                double cur_distance = calculate_distance(_nodelist[next_visit_idx],_nodelist[current_nodeID]);

                if (!found_closed && !found_opened) 
                {
                    // first visit to the node
                    open_nodes.push_back(next_visit_idx);
                    // update values
                     _nodelist[next_visit_idx].g = _nodelist[current_nodeID].g + cur_distance;
                    _nodelist[next_visit_idx].f = _nodelist[next_visit_idx].g + _nodelist[next_visit_idx].h; //  + _nodelist[next_visit_idx].g
                    _nodelist[next_visit_idx].parent_node_ID = current_nodeID;

                }
                else if (!found_closed && found_opened) 
                {
                    // already visited
                    //  check if we should update the table 
                    if(cur_distance + _nodelist[current_nodeID].g < _nodelist[next_visit_idx].f)
                    {
                        // update values if improve
                        _nodelist[next_visit_idx].g = _nodelist[current_nodeID].g + cur_distance;
                        _nodelist[next_visit_idx].f = _nodelist[next_visit_idx].g + _nodelist[next_visit_idx].h; //  + _nodelist[next_visit_idx].g
                        _nodelist[next_visit_idx].parent_node_ID = current_nodeID;
                    }
                }

            }

                // calculate distance
            open_nodes.push_back(current_nodeID+1);
            /*-- End Exercise 2/3 --*/

            // remove the current node from the open list and add it to the closed list
            open_nodes.remove(current_nodeID);
            closed_nodes.push_back(current_nodeID);
        }
    }
    
    /* 2: Trace back the optimal path (if the goal could be reached) */
    if (goal_reached){
        std::list<int> path_node_IDs = {};
        
        /*-- Exercise 3/3: Put the node IDs of nodes in the optimal path (from entrance to finish) in the list path_node_IDs --*/
        // Put your code here
        /*-- End Exercise 3/3 --*/
        std::vector<int> optinm = {_entrance_nodeID};
        Node iterate_node = _nodelist[_finish_nodeID];
        path_node_IDs.push_front(_finish_nodeID);

        int nodeID = _finish_nodeID;
        while (nodeID != _entrance_nodeID)
        {
            // Iterate through the nodes until there is no more parent node
            path_node_IDs.push_front(nodeID);
            nodeID = _nodelist[nodeID].parent_node_ID;
        }
        // add begininig
        path_node_IDs.push_front(nodeID);

        setPathNodeIDs(path_node_IDs);
        setPathFound(true);
    }
}

void Planner::setNodeResolution(const double &node_resolution){
    _node_resolution = node_resolution;
}

void Planner::setNodeList(const std::vector<Node> &nodelist){
    _nodelist = nodelist;
}

void Planner::setEntrance(const int &entrance_nodeID){
    _entrance_nodeID = entrance_nodeID;
}

void Planner::setFinish(const int &finish_nodeID){
    _finish_nodeID = finish_nodeID;
}

void Planner::setConnections(const std::vector<std::vector<int>> &connections){
    _connections = connections;
}

void Planner::setPathFound(const bool &path_found){
    _path_found = path_found;
}

void Planner::setPathNodeIDs(const std::list<int> &path_node_IDs){
    _path_node_IDs = path_node_IDs;
}
