#include <unordered_set>
#include <queue>
#include <functional>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>

#include "include/json.hpp"
#include "include/Map2D.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define FACTOR 50

using json = nlohmann::json;

// This class define the single cell
class GridLocation {
public:
    int x, y;

    inline bool operator==(const GridLocation& other) const {
        return x == other.x && y == other.y;
    }

    inline bool operator!=(const GridLocation& other) const {
        return x != other.x || y != other.y;
    }

    inline bool operator<(const GridLocation& other) const {
        return x < other.x && y < other.y;
    }

    inline bool operator>(const GridLocation& other) const {
        return x > other.x || y > other.y;
    }

    GridLocation(int x_, int y_) : x(x_), y(y_) {};
    GridLocation() : x(0), y(0) {};
};

namespace std {
    // implement hash function so we can put GridLocation into an unordered_set
    template <> struct hash<GridLocation> {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
}


class SquareGrid {
private:
    int size_x, size_y;
    std::unordered_set<GridLocation> walls;

public:
    // Eight connected map
    const std::array<GridLocation, 8> DIRS = {GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1},
                                              GridLocation{1, 1}, GridLocation{-1, -1}, GridLocation{-1, 1}, GridLocation{1, -1}};

    SquareGrid(int size_x_, int size_y_) : size_x(size_x_), size_y(size_y_) {}

    // check if a cell is in the bounds of the map
    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < size_x
               && 0 <= id.y && id.y < size_y;
    }

    // check if a cell is a wall
    bool passable(GridLocation id) const {
        return walls.find(id) == walls.end();
    }

    // create a new wall
    void add_wall(int x, int y) {
        walls.emplace(x, y);
    }


    // return the list of free cells around a specific one
    std::vector<GridLocation> neighbors(GridLocation id) const {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS) {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0) {
            // aesthetic improvement on square grids
            std::reverse(results.begin(), results.end());
        }

        return results;
    }

    // cost to move from a cell to another as the euclidean distance
    double cost(GridLocation from_node, GridLocation to_node) const {
        return std::hypot((from_node.x - to_node.x), (from_node.y - to_node.y));
    }

    // print an ASCII grid
    void draw_grid(std::vector<GridLocation> path) {
        char map[size_x][size_y];

        for(int x = 0; x < size_x; x++) {
            for(int y=0; y < size_y; y++) {
                map[x][y]='.';
            }
        }

        for (const auto& elem : walls) {
            map[elem.x][elem.y] = '#';
        }

        for (const auto& elem : path) {
            map[elem.x][elem.y] = '@';
        }

        for(int x = 0; x < size_x; x++) {
            for(int y=0; y < size_y; y++) {
                std::cout<<map[x][y];
            }
            std::cout<<std::endl;
        }
    }

    // visualise the map with openCV
    void draw_map(std::vector<GridLocation> path) {
        char sim_window[] = "Simulator";

        cv::Mat sim_image(int(std::ceil(size_x * FACTOR)),
                          int(std::ceil(size_y * FACTOR)),
                          CV_8UC3, cv::Scalar( 255, 255, 255 ));

        for (const auto& elem : walls) {
            rectangle(sim_image, cv::Point(elem.x * FACTOR, elem.y * FACTOR), cv::Point((elem.x + 1) * FACTOR, (elem.y + 1) * FACTOR),
                      cv::Scalar( 0, 0, 0 ), cv::FILLED, cv::LINE_8);
        }

        for (const auto& elem : path) {
            circle(sim_image, cv::Point(int(elem.x * FACTOR + FACTOR/2), int(elem.y * FACTOR +  FACTOR/2)),
                   5, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_8);
        }

        cv::imshow(sim_window,sim_image);
        cv::waitKey(0);
    }
};

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<T, priority_t> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(item, priority);
    }

    priority_t get() {
        priority_t best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

// It's ya boi Dijkstra
template<typename Location, typename Graph>
void dijkstra_search(Graph graph, Location start, Location goal,
                     std::unordered_map<Location, Location>& came_from,
                     std::unordered_map<Location, double>& cost_so_far) {
    PriorityQueue<double, Location> frontier;
    frontier.put(0, start);

    std::cout<<"start "<<start.x<<" "<<start.y<<std::endl;
    std::cout<<"goal "<<goal.x<<" "<<goal.y<<std::endl;

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                came_from[next] = current;
                frontier.put(new_cost, next);
            }
        }
    }
}

template<typename Location>
std::vector<Location> reconstruct_path(Location start, Location goal, std::unordered_map<Location, Location> came_from) {
    std::vector<Location> path;
    Location current = goal;
    while (current != start) {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char **argv) {
    // TODO file location is hardcoded
    std::string filename = "/home/gianluca/locations.json";

    std::ifstream file(filename);
    json map_location;
    file >> map_location;

    // TODO check the JSON file, map image location is hardcoded and an absolute path
    // Open the image of the map from the JSON and get the size
    cv::Mat image = cv::imread(map_location["map"]["location"], cv::IMREAD_GRAYSCALE);
    Map2D tmap(map_location["map"]["occupiedThresh"],
               map_location["map"]["freeThresh"],
               image.cols,
               image.rows);

    // load the value of each pixel in a vector
    for(auto it = image.begin<uchar>(); it != image.end<uchar>(); ++it) {
        tmap.cells.push_back(*it);
    }

    SquareGrid local_map(tmap.cellSizeX, tmap.cellSizeY);

    // if a pixel is above a specific threshold, save it as a wall in the gridmap
    std::cout<<"Creating walls"<<std::endl;
    int index = 0;
    for(int y = 0; y < tmap.cellSizeX; y++){
        for(int x = 0; x < tmap.cellSizeY; x++) {
            if (((255 - tmap.cells[index]) / 255.0) > tmap.occupiedThresh) {
                local_map.add_wall(x, y);
            }
            index++;
        }
    }


    // define start and goal position
    GridLocation start(map_location["start"][0], map_location["start"][1]);
    GridLocation goal(map_location["goal"][0], map_location["goal"][1]);

    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;

    std::cout<<"plan"<<std::endl;
    dijkstra_search(local_map, start, goal, came_from, cost_so_far);
    std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
    std::cout<<"done"<<std::endl;

    //show result
    local_map.draw_map(path);

    return true;
}