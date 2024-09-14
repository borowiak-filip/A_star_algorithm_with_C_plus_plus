#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y; 
    float cost;
    float h;
    float total; // Total cost of heurisitc (h) and cost 

    // keep track of parent node as pointer
    Node* parent = nullptr;

    Node(int x_, int y_): x(x_), y(y_), cost(INFINITY), h(0), total(INFINITY) {} // construct: Initializer list

    // less than operator for node struct overload
    bool operator <(const Node& other) const {
        return total > other.total;
    }
};

// custom compare for nodes
struct CompareNode {
    bool operator() (Node* a, Node* b) {
        return a->total > b->total;
    }
};


float heuristic(Node* a, Node* b) {
    return std::abs(a->x - b->x) + std::abs(a->y - b->y); // Manhattan heuristic
}

std::vector<std::vector<int>> map_gen(int size) {

// Create a 2D vector of dim {size x size}
std::vector<std::vector<int>> grid(size, std::vector<int>(size, 1));

// rnd number gen
std::srand(std::time(0));

// allocate obstacles on the map

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j ) {

            // random decision to where the object should be placed

            if (std::rand() % 10 < 2) { // around 20% chance of allocation per array value
            grid[i][j] = 0; // 0 is the obstacle marker and 1s are walkable fields

            // Game starts always in point [0][0]

            grid[0][0] = 1;

            // ends in the corner

            grid[size-1][size-1] = 1;

            }

        }
    }
    return grid;
}
// printing the array

void print_grid(const std::vector<std::vector<int>>& grid) {

    for (const auto& row : grid) {
        for (int cell : row) {
            std::cout << cell << " ";
        }
        std::cout << std::endl;
    }
}


std::vector<Node*> get_neighbours(Node* current, std::vector<std::vector<int>>& grid, int size) {
    std::vector<Node*> neighbours; // all adjecent neighbours for current node
    int x = current->x; // access current node y coord
    int y = current->y; // access current node x coord

    // create a vector for storing all possilbe directions {0, -1}, {0, 1} {-1, 0}, {1, 0} || UP, DOWN, LEFT, RIGHT
    std::vector<std::pair<int, int>> directions = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};

    for (auto dir : directions) {

        int neighbour_x = x + dir.first; // xs are first value
        int neighbour_y = y + dir.second;

        // both coord x and y need to be greater than 0 and less than dim size and array value needs to be 1 if walkable
        if (neighbour_x >= 0 && neighbour_x < size && neighbour_y >= 0 && neighbour_y < size && grid[neighbour_x][neighbour_y] == 1) {
            neighbours.push_back(new Node(neighbour_x, neighbour_y));
        }

    }

    return neighbours;
}

void a_star(Node* start_node, Node* goal_node, std::vector<std::vector<int>>& grid, int size) {

    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_set; 
    std::vector<Node*> closed_set;

    start_node->cost = 0; // set the cost of current node to 0;
    start_node->h = heuristic(start_node, goal_node); // calculate heuristic
    start_node->total = start_node->cost + start_node->h;

    // push the initial node into the queue
    open_set.push(start_node);

    while (!open_set.empty()) {
        Node* current = open_set.top();
        
        open_set.pop();

        // if goal has been reached, get the path
        if (current->x == goal_node->x && current->y == goal_node->y) {
            std::cout << "Path has been found! \n";

            // make sure there isnt empty pointer
            while (current != nullptr) {
                std::cout << "(" << current->x << ", " << current->y << ")";
                current = current->parent; // go back now
            }
            std::cout << std::endl;
            return; 
        }

        // Check all adjecent nodes

        for (Node* neighbour: get_neighbours(current, grid, size)) {

            // calculate cost, assuming cost from (0, 0) to (0, 1) = 1 and so on

            float tentative_g = current->cost + 1;

            // if neighbour has already been explored, skip

            auto n = std::find(closed_set.begin(), closed_set.end(), neighbour);
            if (n != closed_set.end()) {
                continue;
            }

            // if this path is shorter update
            if(tentative_g < neighbour->cost) {
                neighbour->cost = tentative_g;
                neighbour->h = heuristic(neighbour, goal_node);
                neighbour->total = neighbour->cost + neighbour->h;

                neighbour->parent = current; // set the parent

                open_set.push(neighbour); // add neighbour to checked nodes
            }
        }

        closed_set.push_back(current);
    }

    std::cout << "There was no path to be found \n";


}

int main() {

    int size = 30; // dim of the array

    // init grid
    std::vector<std::vector<int>> grid = map_gen(size);

    std::cout << "init grid \n";
    print_grid(grid);

    std::cout << "init start node and end node \n";

    Node start_node = Node(0, 0);
    Node* start_node_ptr = &start_node;

    Node end_node = Node(size-1, size-1);
    Node* end_node_ptr = &end_node;

    a_star(start_node_ptr, end_node_ptr, grid, size);





    return 0;
}


