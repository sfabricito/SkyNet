/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
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
import java.util.Set;
import javax.swing.JFrame;
import javax.swing.JPanel;
import org.jgrapht.Graph;
import org.jgrapht.Graphs;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
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
    
    public static boolean isConnected(Graph<Vertex, DefaultEdge> graph) {
        // Use JGraphT's ConnectivityInspector to check if the graph is connected
        ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        return connectivityInspector.isConnected();
    }
    //Algoritmo para el segundo caso 
    //https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/?ref=lbp
    //O prims algorithm
    /*
    public void runKruskalAlgorithm() {
        // Apply Kruskal's algorithm to find the minimum spanning tree
        KruskalMinimumSpanningTree<Vertex, DefaultEdge> kruskal =
                new KruskalMinimumSpanningTree<>(graph);

        // Get the minimum spanning tree as a graph
        Graph<Vertex, DefaultEdge> minimumSpanningTree = kruskal.getSpanningTree();

        // Print or process the minimum spanning tree as needed
        System.out.println("Minimum Spanning Tree Edges:");
        for (DefaultEdge edge : minimumSpanningTree.edgeSet()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            System.out.println(source + " -- " + target);
        }
    }*/

    
    
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
