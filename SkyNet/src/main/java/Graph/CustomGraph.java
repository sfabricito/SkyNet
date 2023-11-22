
package Graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.graph.SparseGraph;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import javax.swing.JFrame;
import javax.swing.JPanel;
import org.jgrapht.Graph;
import org.jgrapht.Graphs;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import org.jgrapht.alg.interfaces.SpanningTreeAlgorithm;
import org.jgrapht.alg.spanning.KruskalMinimumSpanningTree;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

/**
 *
 * @author fabri
 */
public class CustomGraph {
    final private String FILEPATH = "src/main/java/Data/graph.json";
    private org.jgrapht.Graph<Vertex, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);
    
    public CustomGraph(){
        
    }
    
    public void loadGraph(String route){
        ObjectMapper objectMapper = new ObjectMapper();
        ArrayList<Vertex> nodes = new ArrayList<Vertex>();
        try {
            Vertex[] vertices = objectMapper.readValue(new File(route), Vertex[].class);

            nodes.addAll(Arrays.asList(vertices));            
            for (Vertex vertex : nodes) {
                graph.addVertex(vertex);
            }
            
            for (Vertex vertex : nodes) {
                for (Edge edge : vertex.getEdges()) {
                    Vertex toVertex = searchNodeByName(edge.getToVertex());
                    if (toVertex != null && vertex != toVertex) {
                        graph.addEdge(vertex, toVertex);
                    }
                }
            }

                        
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public void paintGraph(JPanel panel){
        edu.uci.ics.jung.graph.Graph<String, String> jungGraph = convertJGraphTtoJUNG(graph);
        // Create JUNG visualization
        BasicVisualizationServer<String, String> vv = new BasicVisualizationServer<>(new CircleLayout<>(jungGraph));
        vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller());

        // Set edge labels to be displayed
        vv.getRenderContext().setEdgeLabelTransformer(edge -> edge);
        
        JFrame frame = new JFrame("Graph Visualization");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    

        // Display the graph in a JFrame
        panel.removeAll();
        panel.add(vv);
        panel.revalidate();
        panel.repaint();
        
                // Add the graph visualization to the panel
        panel.add(vv);
    }
    
    private static edu.uci.ics.jung.graph.Graph<String, String> convertJGraphTtoJUNG(org.jgrapht.Graph<Vertex, DefaultEdge> jGraphTGraph) {
        edu.uci.ics.jung.graph.Graph<String, String> jungGraph = new SparseGraph<>();

        // Add vertices
        Set<Vertex> vertices = jGraphTGraph.vertexSet();
        for (Vertex vertex : vertices) {
            jungGraph.addVertex(vertex.getVertex());
        }

        // Add edges
        for (Vertex source : vertices) {
            Set<DefaultEdge> outgoingEdges = jGraphTGraph.outgoingEdgesOf(source);
            for (DefaultEdge edge : outgoingEdges) {
                Vertex target = Graphs.getOppositeVertex(jGraphTGraph, edge, source);
                String edgeIdentifier =  target.getVertex() + " ➡ " +  source.getVertex();
                jungGraph.addEdge(edgeIdentifier, source.getVertex(), target.getVertex());
            }
        }

        return jungGraph;
    }
    
    public Vertex searchNodeByName(String vertexName){
        for (Vertex vertex : graph.vertexSet()) {
            if (vertex.getVertex().equals(vertexName)) {
                return vertex;
            }
        }
        return null;
    }


    public Graph<Vertex, DefaultEdge> getGraph() {
        return graph;
    }
    
    public void disconnectGraph() {
        // Check if the graph is already disconnected
        ConnectivityInspector<Vertex, DefaultEdge> initialConnectivityInspector = new ConnectivityInspector<>(graph);
        if (!initialConnectivityInspector.isConnected()) {
            System.out.println("The graph is already disconnected. No vertices will be removed.");
            return;
        }

        Set<Vertex> verticesToRemove = new HashSet<>();

        // Iterate through each vertex in the graph
        for (Vertex vertex : graph.vertexSet()) {
            // Create a temporary graph without the current vertex
            Graph<Vertex, DefaultEdge> tempGraph = new SimpleGraph<>(DefaultEdge.class);
            Set<Vertex> remainingVertices = new HashSet<>(graph.vertexSet());
            remainingVertices.remove(vertex);

            // Add vertices and edges to the temporary graph
            for (Vertex v : remainingVertices) {
                tempGraph.addVertex(v);
                for (DefaultEdge edge : graph.edgesOf(v)) {
                    Vertex oppositeVertex = Graphs.getOppositeVertex(graph, edge, v);
                    if (remainingVertices.contains(oppositeVertex)) {
                        tempGraph.addEdge(v, oppositeVertex);
                    }
                }
            }

            // Check if the temporary graph is connected
            ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(tempGraph);
            if (!connectivityInspector.isConnected()) {
                // If disconnected, add the current vertex to verticesToRemove and stop further attempts
                if (verticesToRemove.isEmpty()) {
                    verticesToRemove.add(vertex);
                }
                break;
            }
        }

        // Remove the identified vertices from the original graph
        for (Vertex v : verticesToRemove) {
            graph.removeVertex(v);
        }
    }
    //Probar con eliminar varios para volver disconexo, se supone que individual funciona bien 
    
    
    
    
    //Algoritmo para el segundo caso 
    //https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/?ref=lbp
    public void visualizeMinimumSpanningTreeByGoods() {
        // Check if the graph is connected
        ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        if (!connectivityInspector.isConnected()) {
            System.out.println("The graph is not connected. Cannot visualize minimum spanning tree.");
            return;
        }

        // Apply Kruskal's algorithm to find the minimum spanning tree based on goods

    KruskalMinimumSpanningTree<Vertex, DefaultEdge> kruskal =
        new KruskalMinimumSpanningTree<>(graph);

    Graph<Vertex, DefaultEdge> minimumSpanningTreeGraph = (Graph<Vertex, DefaultEdge>) kruskal.getSpanningTree();
    List<DefaultEdge> sortedEdges = minimumSpanningTreeGraph
        .edgeSet()
        .stream()
        .sorted(Comparator.comparingDouble(e -> ((Edge) e).getGoods()))
        .collect(Collectors.toList());


        // Get the minimum spanning tree as a graph
        Graph<Vertex, DefaultEdge> minimumSpanningTree = new SimpleGraph<>(DefaultEdge.class);
        Graphs.addAllVertices(minimumSpanningTree, graph.vertexSet());
        for (DefaultEdge edge : kruskal.getSpanningTree().getEdges()) {
            minimumSpanningTree.addEdge(graph.getEdgeSource(edge), graph.getEdgeTarget(edge));
        }

        // Print or process the minimum spanning tree as needed
        System.out.println("Minimum Spanning Tree Edges:");
        for (DefaultEdge edge : minimumSpanningTree.edgeSet()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            System.out.println(source + " -- " + target + " Goods: " + ((Edge) edge).getGoods());
        }

        // Visualize the minimum spanning tree (You can replace this with your own visualization logic)
        visualizeGraph(minimumSpanningTree);

        // Annul the corresponding edges in the original graph
        for (DefaultEdge edge : minimumSpanningTree.edgeSet()) {
            graph.removeEdge(edge);
        }

        // Visualize the graph after annulment (You can replace this with your own visualization logic)
        visualizeGraph(graph);
    }
    
    private void visualizeGraph(Graph<Vertex, DefaultEdge> graphToVisualize) {
        System.out.println("Visualizing Graph:");
        for (DefaultEdge edge : graphToVisualize.edgeSet()) {
            Vertex source = graphToVisualize.getEdgeSource(edge);
            Vertex target = graphToVisualize.getEdgeTarget(edge);
            System.out.println(source + " -- " + target + " Goods: " + ((Edge) edge).getGoods());
        }
    }

    
    //Algoritmo para el tercer caso
    //https://www.geeksforgeeks.org/convert-undirected-connected-graph-to-strongly-connected-directed-graph/
    //Algo similar
    
    //Algoritmo para el cuarto caso
    //Djistra https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/?ref=lbp
    
    //Algoritmo para aniquilacion total quinto caso 
    //https://www.geeksforgeeks.org/euler-circuit-directed-graph/?ref=lbp

    
    //Algoritmo para el sexto caso
    //https://www.geeksforgeeks.org/eulerian-path-and-circuit/
    //Relativo
    
    //Algoritmo para el setimo caso
    //Djistra pero yo escogiendo las ciudades
    
    //Algoritmo para le octavo caso
    //Djistra que sume mas por poder militar
    
    //Algortimo para el noveno caso
    //Djistra pero que retorne todos los caminos y escoger que camino
    
    //Algoritmo para el decimo caso
    //No entendi muy bien
    
    
    
    
    
    
    
    
    
    
    //https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/?ref=lbp
}
