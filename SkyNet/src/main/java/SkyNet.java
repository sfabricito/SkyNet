
import Graph.Edge;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

public class SkyNet {

    public static void main(String[] args) {
        Graph<String, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);

        // Add vertices
        graph.addVertex("A");
        graph.addVertex("B");
        graph.addVertex("C");
        Edge edge = new Edge("hello", 0, 4, 4);
        // Add edges
        graph.addEdge("A", "B", edge);

        DefaultEdge edge1 = graph.getEdge("A", "B");
        
        if (edge1 != null) {
            System.out.println("Military: " + ((Edge) edge1).getMilitary());
        } else {
            System.out.println("No edge found between "  + " and " );
        }

        
    }
}
