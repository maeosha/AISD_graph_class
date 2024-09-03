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

    void walk(const Vertex& start_vertex, std::function<void (const Vertex&)> action, std::set<Vertex>& visited) const {
        action(start_vertex);
        visited.insert(start_vertex);

        for (const auto& edge : graphList.at(start_vertex)) {
            if (visited.find(edge.to) == visited.end()) {
                walk(edge.to, action, visited);
            }
        }
    }

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

    float get_average_dist(const Vertex& vertex) const {
        if (!has_vertex(vertex)) {
            return 0;
        }
        Distance sum_dist = 0;
        size_t count_of_edges = graphList.at(vertex).size();

        if (count_of_edges == 0) {
            return 0;
        }

        for (const auto& edge : graphList.at(vertex)) {
            sum_dist += edge.dist;
        }
        return static_cast<float>(sum_dist) / count_of_edges;
    }

    std::vector<Vertex> vertices() const {
        std::vector<Vertex> vertices;
        for (const auto& [vert, edges] : graphList) {
            vertices.push_back(vert);
        }
        return vertices;
    }

    std::vector<Edge> edges(const Vertex& vertex) const {
        if (!has_vertex(vertex)) {
            return {};
        }

        std::vector<Edge> edges_of_vert;
        for (const auto& [vert, edges] : graphList) {
            for (const auto& edge : edges) {
                if (edge.from == vertex || edge.to == vertex) {
                    edges_of_vert.push_back(edge);
                }
            }
        }

        return edges_of_vert;
    }

    std::vector<Edge> edges() const {
        std::vector<Edge> edges_list;
        for (const auto& [u, edges] : graphList) {
            for (const auto& edge : edges) {
                edges_list.push_back(edge);
            }
        }
        return edges_list;
    }

    size_t order() const {
        return graphList.size();
    }

    size_t degree(const Vertex& vertex) const {
        if (!has_vertex(vertex)) {
            return 0;
        }
        return graphList.at(vertex).size();
    }

    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        std::map<Vertex, Vertex> pred;
        std::map<Vertex, Distance> dist;
        std::map<Vertex, bool> dist_updated;

        size_t updated_count = 0;

        if (!has_vertex(from) || !has_vertex(to)) {
            return {};
        }

        for (const auto& [vert, edge] : graphList) {
            dist[vert] = INFINITY;
            dist_updated[vert] = false;
        }

        dist[from] = 0;
        updated_count++;
        dist_updated[from] = true;

        for (size_t i = 0; i < order(); ++i) {
            for (const auto& [u, edges] : graphList) {
                if (dist_updated[u]) {
                    for (const auto& edge : edges) {
                        if ((edge.dist + dist[u]) < dist[edge.to]) {
                            dist[edge.to] = edge.dist + dist[u];
                            pred[edge.to] = u;
                            if (!dist_updated[edge.to]) {
                                updated_count++;
                                dist_updated[edge.to] = true;
                            }
                        }
                    }
                    dist_updated[u] = false;
                    updated_count--;
                }
                if (updated_count == 0) {
                    break;
                }
            }
        }

        if (updated_count != 0) {
            throw std::runtime_error("Graph contains a negative-weight cycle");
        }

        std::vector<Edge> path;
        Vertex current = to;

        while (current != from) {
            if (pred.find(current) == pred.end()) {
                return {};
            }
            Vertex predecessor = pred[current];
            Distance weight = dist[current] - dist[predecessor];
            path.emplace(path.begin(), predecessor, current, weight);
            current = predecessor;
        }
        return path;
    }

    void walk(const Vertex& start_vertex, std::function<void (const Vertex&)> action) const {
        if (!has_vertex(start_vertex)) {
            return;
        }

        std::set<Vertex> visited;
        walk(start_vertex, action, visited);
    }
};
