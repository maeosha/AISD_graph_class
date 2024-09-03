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

    void add_edge(const Vertex& from, const Vertex& to, const Distance& dist) {
        add_vertex(from);
        add_vertex(to);
        graphList[from].push_back(Edge(from, to, dist));
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (graphList.find(from) != graphList.end()) {
            auto& edges = graphList[from];
            auto it = edges.begin();
            while (it != edges.end()) {
                if (it->from == from && it->to == to) {
                    it = edges.erase(it);
                    return true;
                } else {
                    ++it;
                }
            }
        }

        if (graphList.find(to) != graphList.end()) {
            auto& edges = graphList[to];
            auto it = edges.begin();
            while (it != edges.end()) {
                if (it->from == from && it->to == to) {
                    it = edges.erase(it);
                    return true;
                } else {
                    ++it;
                }
            }
        }

        return false;
    }

    bool remove_edge(const Edge& del_edge) {
        return remove_edge(del_edge.from, del_edge.to);
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) || !has_vertex(to)) {
            return false;
        }

        for (const auto& edge : graphList.at(from)) {
            if (edge.from == from && edge.to == to) {
                return true;
            }
        }

        return false;
    }

    bool has_edge(const Edge& edge) const {
        return has_edge(edge.from, edge.to);
    }
};
