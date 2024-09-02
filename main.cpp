class Graph {
private:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance dist;

        Edge() : from(0), to(0), dist(0) {}

        Edge(const Vertex &from, const Vertex &to, const Distance &dist) {
            this->from = from;
            this->to = to;
            this->dist = dist;
        }

        bool operator==(const Edge &edge) {
            if (from == edge.from && to == edge.to && dist == edge.dist) {
                return true;
            }

            return false;
        }
    };

    std::map <Vertex, std::vector<Edge>> graphList;
};