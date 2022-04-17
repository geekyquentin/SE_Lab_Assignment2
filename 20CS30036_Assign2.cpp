#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include "rapidxml.hpp"

using namespace std;
using namespace rapidxml;

#define ll long long int
#define ln "\n"
#define R 6371.0  // radius of earth in km

class Node {
    ll id;
    string name;
    double longitude;
    double latitude;
    int index;

   public:
    Node(ll id, double longitude, double latitude, int index) {
        this->id = id;
        this->longitude = longitude;
        this->latitude = latitude;
        this->index = index;
    }

    // Set name
    void SetName(string name) { this->name = name; }

    // Get index
    int GetIndex() { return this->index; }

    // Distance from current node
    double DistanceFromCurrentNode(Node node) {
        // Haversine formula
        double dlon = (this->longitude - node.longitude) * M_PI / 180.0;
        double dlat = (this->latitude - node.latitude) * M_PI / 180.0;

        double a = pow(sin(dlat / 2.0), 2) + cos(this->latitude * M_PI / 180.0) * cos(node.latitude * M_PI / 180.0) * pow(sin(dlon / 2.0), 2);

        return 2 * R * asin(sqrt(a));
    }

    // Print node information
    void PrintNodeInfo() {
        cout << "ID: " << this->id << ln;
        // Print name only if it exists
        if (this->name != "") {
            cout << "Name: " << this->name << ln;
        }
        cout << "Longitude: " << this->longitude << ln;
        cout << "Latitude: " << this->latitude << ln;
    }

    // check if the passed string is a substring of the current node's name
    bool IsValidSubstr(string substr) {
        if (this->name.find(substr) != string::npos) {
            return true;
        }

        return false;
    }
};

ll GetNumberOfNodes(xml_node<>* root_node, unordered_map<ll, Node>& node_map, unordered_map<int, ll>& index_map);
ll GetNumberOfWays(xml_node<>* root_node, unordered_map<ll, Node>& node_map, vector<vector<pair<int, double>>>& adj_list);
void GetNodesWithSubstring(unordered_map<ll, Node>& node_map);
void GetKNearestNodes(unordered_map<ll, Node>& node_map);
void GetShortestPath(vector<vector<pair<int, double>>>& adj_list, unordered_map<ll, Node>& node_map, unordered_map<int, ll>& index_map);
double Dijkstra(vector<vector<pair<int, double>>>& adj_list, int source, int destination, unordered_map<int, ll>& index_map);
void PrintPath(vector<int>& parent, int source, int destination, unordered_map<int, ll>& index_map);
void DisplayInstructions();

int main() {
    ifstream file("map.osm");
    xml_document<> doc;

    // Read the xml file into a vector
    vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    buffer.push_back('\0');

    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

    xml_node<>* root_node = doc.first_node();

    // map of node id to node
    unordered_map<ll, Node> node_map;
    // map of index to node id
    unordered_map<int, ll> index_map;

    ll nodes = GetNumberOfNodes(root_node, node_map, index_map);
    cout << "Number of nodes: " << nodes << ln;

    // adjacency list of nodes with their distances and indexes
    vector<vector<pair<int, double>>> adj_list(nodes);

    ll ways = GetNumberOfWays(root_node, node_map, adj_list);
    cout << "Number of ways: " << ways << ln;

    DisplayInstructions();

    while (true) {
        cout << "Enter your choice: ";
        int input;
        cin >> input;

        switch (input) {
            case 1:
                GetNodesWithSubstring(node_map);
                break;
            case 2:
                GetKNearestNodes(node_map);
                break;
            case 3:
                GetShortestPath(adj_list, node_map, index_map);
                break;
            case 0:
                return 0;
            default:
                cout << "Invalid choice, enter again" << ln << ln;
        }
    }

    return 0;
}

ll GetNumberOfNodes(xml_node<>* root_node, unordered_map<ll, Node>& node_map, unordered_map<int, ll>& index_map) {
    ll nodes = 0;

    // Iterate over all the nodes
    for (xml_node<>* node = root_node->first_node("node"); node; node = node->next_sibling("node")) {
        // Get the node id, name and coordinates
        ll id = stoll(node->first_attribute("id")->value());
        double longitude = stod(node->first_attribute("lon")->value());
        double latitude = stod(node->first_attribute("lat")->value());
        int index = nodes;

        // Add the node to the map
        node_map[id] = Node(id, longitude, latitude, index);

        // Add the index to the map
        index_map[index] = id;

        // Check if the node has a name
        if (node->first_node() != NULL) {
            for (xml_node<>* q_value = node->first_node("tag"); q_value; q_value = q_value->next_sibling("tag")) {
                string k_value = q_value->first_attribute("k")->value();
                if (k_value == "name") {
                    // Set the name of the node
                    node_map[id].SetName(q_value->first_attribute("v")->value());
                    break;
                }
            }
        }

        nodes++;
    }

    return nodes;
}

ll GetNumberOfWays(xml_node<>* root_node, unordered_map<ll, Node>& node_map, vector<vector<pair<int, double>>>& adj_list) {
    ll ways = 0;
    ll prev_id = 0;

    // Iterate over all the ways
    for (xml_node<>* way = root_node->first_node("way"); way; way = way->next_sibling("way")) {
        // Iterate over all the nodes in the way
        for (xml_node<>* node = way->first_node("nd"); node; node = node->next_sibling("nd")) {
            ll curr_id = stoll(node->first_attribute("ref")->value());

            if (prev_id) {
                // Add the edge to the adjacency list
                adj_list[node_map[prev_id].GetIndex()].push_back(make_pair(node_map[curr_id].GetIndex(), node_map[prev_id].DistanceFromCurrentNode(node_map[curr_id])));
                adj_list[node_map[curr_id].GetIndex()].push_back(make_pair(node_map[prev_id].GetIndex(), node_map[curr_id].DistanceFromCurrentNode(node_map[prev_id])));
            }

            prev_id = curr_id;
        }
        ways++;
    }

    return ways;
}

void GetNodesWithSubstring(unordered_map<ll, Node>& node_map) {
    cout << "Enter the substring: ";
    string substr;
    cin >> substr;

    ll count = 0;
    // Iterate over all the nodes
    for (auto it = node_map.begin(); it != node_map.end(); it++) {
        // Check if the node has the substring and print it
        if (it->second.IsValidSubstr(substr)) {
            cout << ln << "Node " << ++count << ": " << ln;
            it->second.PrintNodeInfo();
        }
    }

    if (count == 0) {
        cout << "No nodes found" << ln;
    }

    cout << ln;
}

void GetKNearestNodes(unordered_map<ll, Node>& node_map) {
    cout << "Enter the node ID: ";
    ll id;
    cin >> id;

    // Check if the node exists
    if (node_map.find(id) != node_map.end()) {
        cout << "Enter the number of nearest nodes to be printed: ";
        ll k;
        cin >> k;

        vector<pair<Node, double>> distances;

        // Get the distance from the current node to all other nodes
        for (auto it = node_map.begin(); it != node_map.end(); it++) {
            if (it->first != id) {
                double distance = node_map[id].DistanceFromCurrentNode(it->second);
                distances.push_back(make_pair(it->second, distance));
            }
        }

        // Sort the distances
        sort(distances.begin(), distances.end(), [](pair<Node, double>& a, pair<Node, double>& b) {
            return a.second < b.second;
        });

        // Print the k nearest nodes
        cout << ln << "Nearest nodes: ";
        for (int i = 0; i < k; i++) {
            cout << ln << "Node " << i + 1 << ": " << ln;
            distances[i].first.PrintNodeInfo();
            cout << "Distance: " << distances[i].second << " km" << ln;
        }
    } else {
        cout << "Node with ID " << id << " not found" << ln;
    }

    cout << ln;
}

void GetShortestPath(vector<vector<pair<int, double>>>& adj_list, unordered_map<ll, Node>& node_map, unordered_map<int, ll>& index_map) {
    cout << "Enter ID of source node: ";
    ll source_id;
    cin >> source_id;

    cout << "Enter ID of destination node: ";
    ll destination_id;
    cin >> destination_id;

    // Check if the source_id and destination_id nodes exist
    if (node_map.find(source_id) != node_map.end() && node_map.find(destination_id) != node_map.end()) {
        // Get the shortest path using Dijkstra's algorithm
        double shortest_path = Dijkstra(adj_list, node_map[source_id].GetIndex(), node_map[destination_id].GetIndex(), index_map);

        // Print the shortest path
        if (shortest_path != -1) {
            cout << ln << "Shortest path: " << shortest_path << " km" << ln;
        } else {
            cout << ln << "No path found" << ln;
        }
    } else {
        cout << "Source or destination node not found" << ln;
    }

    cout << ln;
}

double Dijkstra(vector<vector<pair<int, double>>>& adj_list, int source, int destination, unordered_map<int, ll>& index_map) {
    // Initialize the distance array
    vector<double> distance(adj_list.size(), numeric_limits<double>::max());

    // Initialize the visited array
    vector<bool> visited(adj_list.size(), false);

    // Initialize the parent array
    vector<int> parent(adj_list.size(), -1);

    // Initialize the priority queue
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    // Initialize the distance of the source node to 0
    distance[source] = 0;

    // Add the source node to the priority queue
    pq.push(make_pair(distance[source], source));

    // Iterate over the priority queue
    while (!pq.empty()) {
        // Get the node with the minimum distance
        int u = pq.top().second;
        pq.pop();

        // Check if the node has already been visited
        if (!visited[u]) {
            // Mark the node as visited
            visited[u] = true;

            // Iterate over the adjacent nodes of the node
            for (auto it = adj_list[u].begin(); it != adj_list[u].end(); it++) {
                // Get the adjacent node
                int v = it->first;

                // Get the distance from the source node to the adjacent node
                double alt = distance[u] + it->second;

                // Check if the distance is less than the current distance
                if (alt < distance[v]) {
                    // Update the distance
                    distance[v] = alt;

                    // Update the parent
                    parent[v] = u;

                    // Add the node to the priority queue
                    pq.push(make_pair(distance[v], v));
                }
            }
        }
    }

    // Get the shortest path distance
    if (distance[destination] == numeric_limits<double>::max()) {
        return -1;
    } else {
        // Print the shortest path
        PrintPath(parent, source, destination, index_map);

        return distance[destination];
    }
}

void PrintPath(vector<int>& parent, int source, int destination, unordered_map<int, ll>& index_map) {
    // Initialize the path vector
    vector<int> path;

    // Get the path
    int curr_id = destination;
    while (curr_id != source) {
        path.push_back(curr_id);
        curr_id = parent[curr_id];
    }

    // Print the path
    cout << "Path:" << ln;
    cout << index_map[source];
    for (int i = path.size() - 1; i >= 0; i--) {
        cout << " - " << index_map[path[i]];
    }
    cout << ln;
}

void DisplayInstructions() {
    cout << ln;
    cout << "Enter the following commands:" << ln;
    cout << "1. Enter the substring to search for" << ln;
    cout << "2. Enter the node id to find the nearest nodes" << ln;
    cout << "3. Enter the source and destination node id to find the shortest path" << ln;
    cout << "0. Exit" << ln;
    cout << ln;
}