#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <functional>

#define INFINITY INT_MAX

template <typename Vertex, typename Distance = int>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance dist;

        Edge() : from(0), to(0), dist(0) {}
        Edge(const Vertex& from, const Vertex& to, const Distance& dist) : from(from), to(to), dist(dist) {}

        bool operator==(const Edge& edge) const {
            return from == edge.from && to == edge.to && dist == edge.dist;
        }
    };

private:
    std::map<Vertex, std::vector<Edge>> graphList;

public:
    bool has_vertex(const Vertex& vertex) const {
        return graphList.find(vertex) != graphList.end();
    }

    void add_vertex(const Vertex& vertex) {
        if (!has_vertex(vertex)) {
            graphList[vertex] = std::vector<Edge>();
        }
    }

    bool remove_vertex(const Vertex& vertex) {
        if (!has_vertex(vertex)) {
            return false;
        }

        for (auto& [vert, edges] : graphList) {
            auto edge = edges.begin();
            while (edge != edges.end()) {
                if (edge->to == vertex || edge->from == vertex) {
                    edge = edges.erase(edge);
                } else {
                    ++edge;
                }
            }
        }

        graphList.erase(vertex);
        return true;
    }
};
