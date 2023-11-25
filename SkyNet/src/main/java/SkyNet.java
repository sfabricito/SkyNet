import Graph.Edge;
import Graph.Vertex;
import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import org.jgrapht.alg.cycle.HierholzerEulerianCycle;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.jgrapht.graph.DefaultDirectedGraph;

public class SkyNet {

    public static void main(String[] args) {
        // Create a graph
        Graph<Vertex, DefaultEdge> graph = new DefaultDirectedGraph<>(DefaultEdge.class);

        Vertex vertex1 = new Vertex("1", 3, 2, 4);
        Vertex vertex2 = new Vertex("2", 3, 2, 4);
        Vertex vertex3 = new Vertex("3", 3, 2, 4);

        graph.addVertex(vertex1);
        graph.addVertex(vertex2);
        graph.addVertex(vertex3);

        // Add edges to make it impossible to form an Eulerian circuit
        Edge edge1 = new Edge("2", 3, 3, 3);
        Edge edge2 = new Edge("3", 3, 3, 3);
        Edge edge3 = new Edge("1", 3, 3, 3);
        graph.addEdge(vertex1, vertex2, edge1);
        graph.addEdge(vertex2, vertex3, edge2);
        graph.addEdge(vertex3, vertex1, edge3);

        // Print the graph
        System.out.println("Graph: " + graph);

        ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        // Check if the graph is connected
        if (!connectivityInspector.isConnected()) {
            System.out.println("The graph is not connected. Total annihilation not possible.");
            //window.AnnihilationNotPosibleNotConnected();
            return;
        }

        // Check if the degrees are suitable for Eulerian circuit
        if (!checkDegreesForEulerianCircuit(graph)) {
            System.out.println("In-degree and out-degree are not equal for each vertex. Total annihilation not possible.");
            //window.AnnihilationNotPosibleNotEven();
            return;
        }

        // Find Eulerian Circuit
        HierholzerEulerianCycle<Vertex, DefaultEdge> eulerianCycle =
                new HierholzerEulerianCycle<>(); // Explicitly specify types here
        List<DefaultEdge> edgeList = eulerianCycle.getEulerianCycle(graph).getEdgeList();

        // Count visits for each vertex
        // Count visits for each vertex
        Map<Vertex, Integer> visitCount = new HashMap<>();
        for (DefaultEdge edge : edgeList) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);

            // Increment visit count for the source and target vertices
            visitCount.put(source, visitCount.getOrDefault(source, 0) + 1);
            visitCount.put(target, visitCount.getOrDefault(target, 0) + 1);
        }


        // Print visit counts for each vertex
        System.out.println("Visit Counts:");
        for (Map.Entry<Vertex, Integer> entry : visitCount.entrySet()) {
            System.out.println(entry.getKey().getVertex() + ": " + entry.getValue());
        }
    }

    private static boolean checkDegreesForEulerianCircuit(Graph<Vertex, DefaultEdge> graph) {
        for (Vertex vertex : graph.vertexSet()) {
            int degree = graph.inDegreeOf(vertex) + graph.outDegreeOf(vertex);
            if (degree % 2 != 0) {
                return false;
            }
        }
        return true;
    }
}
