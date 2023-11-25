import Graph.Edge;
import Graph.Vertex;
import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import org.jgrapht.alg.cycle.HierholzerEulerianCycle;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;

import java.util.List;
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
        //graph.addEdge(vertex3, vertex1, edge3);

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

        // Print edges in the process
        System.out.println("Eulerian Circuit:");
        for (DefaultEdge edge : edgeList) {
            System.out.println(graph.getEdgeSource(edge).getVertex() + " -> " + graph.getEdgeTarget(edge).getVertex());
        }
    }
    
    private static boolean checkDegreesForEulerianCircuit(Graph<Vertex, DefaultEdge> graph) {
        for (Vertex vertex : graph.vertexSet()) {
            int degree = graph.degreeOf(vertex);
            if (degree % 2 != 0) {
                return false;
            }
        }
        return true;
    }
}
