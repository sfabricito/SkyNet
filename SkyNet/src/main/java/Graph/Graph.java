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
            // Lee el archivo JSON y mapea los datos a un array de objetos Vertex
            Vertex[] vertices = objectMapper.readValue(new File(FILEPATH), Vertex[].class);

            // Agrega los vértices al grafo
            nodes.addAll(Arrays.asList(vertices));            
            for (Vertex vertex : nodes) {
                graph.addVertex(vertex);
            }
            
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        for (Vertex node : nodes) {
            System.out.println(node.getVertex());
        }
    }
}
