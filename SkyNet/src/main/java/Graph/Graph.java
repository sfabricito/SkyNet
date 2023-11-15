/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package Graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

/**
 *
 * @author fabri
 */
public class Graph {
    final private String FILEPATH = "src/main/java/Data/graph.json";
    private org.jgrapht.Graph<Vertex, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);
    
    public Graph(){
        
    }
    
    public void loadGraph(){
        ObjectMapper objectMapper = new ObjectMapper();
        ArrayList<Vertex> nodes = new ArrayList<Vertex>();
        try {
            Vertex[] vertices = objectMapper.readValue(new File(FILEPATH), Vertex[].class);

            nodes.addAll(Arrays.asList(vertices));            
            for (Vertex vertex : nodes) {
                graph.addVertex(vertex);
            }
            
            for (Vertex vertex : nodes) {
                for (Edge edge : vertex.getEdges()) {
                    Vertex toVertex = searchNodeByName(edge.getToVertex());
                    if (toVertex != null && vertex != toVertex) {
                        System.out.println("From: " + vertex.getVertex() + " To: " + toVertex.getVertex());
                        graph.addEdge(vertex, toVertex);
                    }
                }
            }

                        
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public Vertex searchNodeByName(String vertexName){
        for (Vertex vertex : graph.vertexSet()) {
            if (vertex.getVertex().equals(vertexName)) {
                return vertex;
            }
        }
        System.out.println("Node: " + vertexName);
        return null;
    }
}
