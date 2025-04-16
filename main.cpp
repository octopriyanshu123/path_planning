#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int GRID_SIZE = 20;
const int GRID_WIDTH = WINDOW_WIDTH / GRID_SIZE;
const int GRID_HEIGHT = WINDOW_HEIGHT / GRID_SIZE;

enum CellType {
    EMPTY,
    OBSTACLE,
    START,
    GOAL,
    PATH,
    EXPLORED
};

struct Node {
    sf::Vector2i position;
    std::shared_ptr<Node> parent;
    float cost;

    Node(sf::Vector2i pos) : position(pos), parent(nullptr), cost(0.0f) {}
};

class Grid {
public:
    Grid() {
        grid.resize(GRID_HEIGHT, std::vector<CellType>(GRID_WIDTH, EMPTY));
    }

    void setCell(int x, int y, CellType type) {
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            grid[y][x] = type;
        }
    }

    CellType getCell(int x, int y) const {
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            return grid[y][x];
        }
        return OBSTACLE; // Treat out of bounds as obstacle
    }

    void generateObstacles(float obstacleProbability = 0.2) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                if (dis(gen) < obstacleProbability) {
                    grid[y][x] = OBSTACLE;
                }
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        sf::RectangleShape cell(sf::Vector2f(GRID_SIZE, GRID_SIZE));
        cell.setOutlineThickness(1);
        cell.setOutlineColor(sf::Color(50, 50, 50));

        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                cell.setPosition(x * GRID_SIZE, y * GRID_SIZE);

                switch (grid[y][x]) {
                    case EMPTY:
                        cell.setFillColor(sf::Color::White);
                        break;
                    case OBSTACLE:
                        cell.setFillColor(sf::Color::Black);
                        break;
                    case START:
                        cell.setFillColor(sf::Color::Green);
                        break;
                    case GOAL:
                        cell.setFillColor(sf::Color::Red);
                        break;
                    case PATH:
                        cell.setFillColor(sf::Color::Blue);
                        break;
                    case EXPLORED:
                        cell.setFillColor(sf::Color(200, 200, 255));
                        break;
                }

                window.draw(cell);
            }
        }
    }

private:
    std::vector<std::vector<CellType>> grid;
};

class RRTStar {
public:
    RRTStar(Grid* grid) : grid(grid), stepSize(3.0f), searchRadius(50.0f), goalReached(false) {
        std::srand(std::time(nullptr));
    }

    void setStart(sf::Vector2i start) {
        startPos = start;
        nodes.clear();
        nodes.push_back(std::make_shared<Node>(start));
        goalReached = false;
        grid.setCell(start.x, start.y, START);
    }

    void setGoal(sf::Vector2i goal) {
        goalPos = goal;
        grid.setCell(goal.x, goal.y, GOAL);
    }

    void setStepSize(float size) { stepSize = size; }
    void setSearchRadius(float radius) { searchRadius = radius; }

    std::vector<sf::Vector2i> findPath(int maxIterations = 1000) {
        for (int i = 0; i < maxIterations && !goalReached; ++i) {
            sf::Vector2i randPoint = getRandomPoint();
            auto nearestNode = findNearestNode(randPoint);
            sf::Vector2i newPoint = steer(nearestNode->position, randPoint);

            if (isCollisionFree(nearestNode->position, newPoint)) {
                auto newNode = std::make_shared<Node>(newPoint);
                newNode->parent = nearestNode;
                newNode->cost = calculateCost(newNode);

                auto nearNodes = findNearNodes(newNode);

                // Choose best parent
                for (const auto& nearNode : nearNodes) {
                    if (isCollisionFree(nearNode->position, newNode->position)) {
                        float newCost = nearNode->cost + getDistance(nearNode->position, newNode->position);
                        if (newCost < newNode->cost) {
                            newNode->parent = nearNode;
                            newNode->cost = newCost;
                        }
                    }
                }

                nodes.push_back(newNode);
                grid.setCell(newPoint.x, newPoint.y, EXPLORED);

                // Rewire tree
                rewire(newNode, nearNodes);

                // Check if goal is reached
                if (getDistance(newPoint, goalPos) <= stepSize && isCollisionFree(newPoint, goalPos)) {
                    goalReached = true;
                    auto goalNode = std::make_shared<Node>(goalPos);
                    goalNode->parent = newNode;
                    goalNode->cost = calculateCost(goalNode);
                    nodes.push_back(goalNode);
                }
            }
        }

        // Reconstruct path
        std::vector<sf::Vector2i> path;
        if (goalReached) {
            auto node = nodes.back(); // Goal node
            while (node != nullptr) {
                path.push_back(node->position);
                grid.setCell(node->position.x, node->position.y, PATH);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        return path;
    }

    void drawTree(sf::RenderWindow& window) {
        sf::VertexArray lines(sf::Lines);
        
        for (const auto& node : nodes) {
            if (node->parent) {
                lines.append(sf::Vertex(
                    sf::Vector2f(node->position.x * GRID_SIZE + GRID_SIZE/2, 
                                node->position.y * GRID_SIZE + GRID_SIZE/2), 
                    sf::Color::Green));
                lines.append(sf::Vertex(
                    sf::Vector2f(node->parent->position.x * GRID_SIZE + GRID_SIZE/2, 
                                node->parent->position.y * GRID_SIZE + GRID_SIZE/2), 
                    sf::Color::Green));
            }
        }

        window.draw(lines);
    }

private:
    Grid& grid;
    std::vector<std::shared_ptr<Node>> nodes;
    sf::Vector2i startPos;
    sf::Vector2i goalPos;
    float stepSize;
    float searchRadius;
    bool goalReached;

    sf::Vector2i getRandomPoint() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> xDist(0, GRID_WIDTH - 1);
        std::uniform_int_distribution<> yDist(0, GRID_HEIGHT - 1);

        // With 10% probability, sample the goal point
        std::uniform_real_distribution<> goalDist(0.0, 1.0);
        if (goalDist(gen) < 0.1) {
            return goalPos;
        }

        return sf::Vector2i(xDist(gen), yDist(gen));
    }

    std::shared_ptr<Node> findNearestNode(const sf::Vector2i& point) {
        std::shared_ptr<Node> nearest = nullptr;
        float minDist = std::numeric_limits<float>::max();

        for (const auto& node : nodes) {
            float dist = getDistance(node->position, point);
            if (dist < minDist) {
                minDist = dist;
                nearest = node;
            }
        }

        return nearest;
    }

    sf::Vector2i steer(const sf::Vector2i& from, const sf::Vector2i& to) {
        float dist = getDistance(from, to);

        if (dist <= stepSize) {
            return to;
        } else {
            float ratio = stepSize / dist;
            return sf::Vector2i(
                from.x + static_cast<int>((to.x - from.x) * ratio),
                from.y + static_cast<int>((to.y - from.y) * ratio)
            );
        }
    }

    bool isCollisionFree(const sf::Vector2i& from, const sf::Vector2i& to) {
        // Bresenham's line algorithm to check all cells along the path
        int x0 = from.x, y0 = from.y;
        int x1 = to.x, y1 = to.y;

        int dx = abs(x1 - x0);
        int dy = -abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            if (grid.getCell(x0, y0) == OBSTACLE) {
                return false;
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        return true;
    }

    std::vector<std::shared_ptr<Node>> findNearNodes(const std::shared_ptr<Node>& newNode) {
        std::vector<std::shared_ptr<Node>> nearNodes;
        float radius = searchRadius * std::sqrt(std::log(nodes.size()) / nodes.size());

        for (const auto& node : nodes) {
            if (getDistance(node->position, newNode->position) <= radius) {
                nearNodes.push_back(node);
            }
        }

        return nearNodes;
    }

    void rewire(std::shared_ptr<Node>& newNode, const std::vector<std::shared_ptr<Node>>& nearNodes) {
        for (const auto& nearNode : nearNodes) {
            if (nearNode != newNode->parent && isCollisionFree(newNode->position, nearNode->position)) {
                float newCost = newNode->cost + getDistance(newNode->position, nearNode->position);
                if (newCost < nearNode->cost) {
                    nearNode->parent = newNode;
                    nearNode->cost = newCost;
                }
            }
        }
    }

    float calculateCost(const std::shared_ptr<Node>& node) {
        if (node->parent == nullptr) {
            return 0.0f;
        }
        return node->parent->cost + getDistance(node->parent->position, node->position);
    }

    float getDistance(const sf::Vector2i& a, const sf::Vector2i& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "RRT* Path Planning");
    window.setFramerateLimit(60);

    Grid grid;
    grid.generateObstacles(0.25); // 25% obstacle probability

    RRTStar rrt(grid);
    rrt.setStart(sf::Vector2i(2, 2));
    rrt.setGoal(sf::Vector2i(GRID_WIDTH - 3, GRID_HEIGHT - 3));
    rrt.setStepSize(3.0f);
    rrt.setSearchRadius(50.0f);

    bool pathFound = false;
    std::vector<sf::Vector2i> path;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space && !pathFound) {
                    path = rrt.findPath(500); // 500 iterations per step
                    if (!path.empty()) {
                        pathFound = true;
                        std::cout << "Path found with " << path.size() << " steps!\n";
                    }
                }
                if (event.key.code == sf::Keyboard::R) {
                    // Reset
                    grid = Grid();
                    grid.generateObstacles(0.25);
                    rrt = RRTStar(grid);
                    rrt.setStart(sf::Vector2i(2, 2));
                    rrt.setGoal(sf::Vector2i(GRID_WIDTH - 3, GRID_HEIGHT - 3));
                    rrt.setStepSize(3.0f);
                    rrt.setSearchRadius(50.0f);
                    pathFound = false;
                    path.clear();
                }
            }
        }

        window.clear(sf::Color::White);
        grid.draw(window);
        rrt.drawTree(window);

        // Draw path
        if (!path.empty()) {
            sf::VertexArray pathLines(sf::LineStrip);
            for (const auto& point : path) {
                pathLines.append(sf::Vertex(
                    sf::Vector2f(point.x * GRID_SIZE + GRID_SIZE/2, 
                                point.y * GRID_SIZE + GRID_SIZE/2), 
                    sf::Color::Red));
            }
            window.draw(pathLines);
        }

        window.display();
    }

    return 0;
}