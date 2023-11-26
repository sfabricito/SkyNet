import Graph.Edge;
import Graph.Vertex;
import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

public class SkyNet {
    public static void main(String[] args) {

        Graph<Vertex, DefaultEdge> graph = new SimpleWeightedGraph<>(DefaultEdge.class);

        // Add vertices
        Vertex v1 = new Vertex("V1", 342, 234, 12);
        graph.addVertex(v1);
        Vertex v2 = new Vertex("V2", 342, 234, 12);
        graph.addVertex(v2);
        Vertex v3 = new Vertex("V3", 342, 234, 12);
        graph.addVertex(v3);

        // Add edges
        Edge e1 = new Edge("V2", 43, 3123, 12);
        graph.addEdge(v1, v2, e1);
        Edge e2 = new Edge("V3", 43, 3123, 13);
        graph.addEdge(v2, v3, e2);
        Edge e3 = new Edge("V1", 43, 3123, 14);
        graph.addEdge(v3, v1, e3);

        Graph<Vertex, DefaultWeightedEdge> graphWeight = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

        // Copy vertices from graph to graphWeight
        for (Vertex vertex : graph.vertexSet()) {
            graphWeight.addVertex(vertex);
        }

        // Copy edges with weight 1 from graph to graphWeight
        for (DefaultEdge edge : graph.edgeSet()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);

            // Add a weighted edge with a weight of 1 to the new graph
            DefaultWeightedEdge weightedEdge = graphWeight.addEdge(source, target);
            graphWeight.setEdgeWeight(weightedEdge, ((Edge) edge).getDistance());
        }

        for (Vertex vertex : graph.vertexSet()) {
            System.out.println("Hello " + vertex.getVertex());
        }

        // Calculate and display the total weight of graphWeight
        double totalWeight = 0.0;
        for (DefaultWeightedEdge edge : graphWeight.edgeSet()) {
            System.out.println("Distance: " + graphWeight.getEdgeWeight(edge));
            totalWeight += graphWeight.getEdgeWeight(edge);
        }
        System.out.println("Total weight of graphWeight: " + totalWeight);
    }
}
