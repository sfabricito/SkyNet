import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

public class SkyNet {
    public static void main(String[] args) {
        // Create the original graph
        Graph<String, DefaultEdge> originalGraph = new SimpleGraph<>(DefaultEdge.class);
        originalGraph.addVertex("A");
        originalGraph.addVertex("B");
        originalGraph.addVertex("C");
        originalGraph.addVertex("D");
        originalGraph.addEdge("A", "B");
        originalGraph.addEdge("B", "C");
        originalGraph.addEdge("C", "D");

        // Create the modified graph
        Graph<String, DefaultEdge> modifiedGraph = new SimpleGraph<>(DefaultEdge.class);
        modifiedGraph.addVertex("A");
        modifiedGraph.addVertex("B");
        modifiedGraph.addVertex("C");
        modifiedGraph.addVertex("D");
        modifiedGraph.addEdge("B", "C");
        // "B" is not added in the modified graph

        // Create a new graph containing the elements in the original graph but not in the modified graph
        Graph<String, DefaultEdge> newGraph = getGraphDifference(originalGraph, modifiedGraph);

        // Print the original graph
        System.out.println("Original Graph: " + originalGraph);

        // Print the modified graph
        System.out.println("Modified Graph: " + modifiedGraph);

        // Print the new graph
        System.out.println("New Graph: " + newGraph);
    }

private static Graph<String, DefaultEdge> getGraphDifference(
        Graph<String, DefaultEdge> originalGraph, Graph<String, DefaultEdge> modifiedGraph) {
    Graph<String, DefaultEdge> differenceGraph = new SimpleGraph<>(DefaultEdge.class);

    // Add all vertices from the original graph to the differenceGraph
    for (String vertex : originalGraph.vertexSet()) {
        differenceGraph.addVertex(vertex);
    }

    // Add edges that are in the original graph but not in the modified graph
    for (DefaultEdge edge : originalGraph.edgeSet()) {
        String source = originalGraph.getEdgeSource(edge);
        String target = originalGraph.getEdgeTarget(edge);

        // Add the edge only if it is not present in the modified graph
        if (!modifiedGraph.containsEdge(source, target)) {
            differenceGraph.addEdge(source, target);
        }
    }

    return differenceGraph;
}




}
