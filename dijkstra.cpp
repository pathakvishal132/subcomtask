#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <stack>

using namespace std;

#define INF INT_MAX

// Data structure to represent a weighted edge
struct Edge
{
    int destination;
    int weight;
};

// Data structure to represent a graph
class Graph
{
public:
    int vertices;
    vector<vector<Edge>> adjacencyList;

    Graph(int v) : vertices(v), adjacencyList(v) {}

    // Function to add an edge to the graph
    void addEdge(int source, int destination, int weight)
    {
        Edge edge1 = {destination, weight};
        Edge edge2 = {source, weight};
        adjacencyList[source].push_back(edge1);
        adjacencyList[destination].push_back(edge2);
    }

    // Function to perform Dijkstra's algorithm
    vector<int> dijkstra(int source)
    {
        vector<int> distance(vertices, INF);
        distance[source] = 0;

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, source});

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            for (const Edge &edge : adjacencyList[u])
            {
                int v = edge.destination;
                int weight = edge.weight;

                if (distance[u] + weight < distance[v])
                {
                    distance[v] = distance[u] + weight;
                    pq.push({distance[v], v});
                }
            }
        }

        return distance;
    }

    // Function to print the shortest path from source to destination
    void printShortestPath(int source, int destination, const vector<int> &distances)
    {
        stack<int> path;
        int current = destination;

        while (current != source)
        {
            path.push(current);
            current = distances[current];
        }

        cout << "Shortest path from " << source << " to " << destination << ": ";
        cout << source << " ";
        while (!path.empty())
        {
            cout << "-> " << path.top() << " ";
            path.pop();
        }
        cout << "\n";
    }
};

int main()
{
    int vertices = 5;
    Graph graph(vertices);

    graph.addEdge(0, 1, 2);
    graph.addEdge(0, 3, 1);
    graph.addEdge(1, 2, 3);
    graph.addEdge(1, 3, 2);
    graph.addEdge(3, 4, 4);
    graph.addEdge(4, 2, 1);

    int source = 0;
    vector<int> shortestDistances = graph.dijkstra(source);

    cout << "Shortest distances from source " << source << ":\n";
    for (int i = 0; i < vertices; ++i)
    {
        cout << "Vertex " << i << ": " << shortestDistances[i] << "\n";
    }

    // Print shortest paths
    for (int i = 0; i < vertices; ++i)
    {
        if (i != source)
        {
            graph.printShortestPath(source, i, shortestDistances);
        }
    }

    return 0;
}
