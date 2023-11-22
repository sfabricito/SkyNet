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
import org.jgrapht.Graph;
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

    //Algoritmo para el primer caso
    //https://www.geeksforgeeks.org/java-program-to-check-whether-undirected-graph-is-connected-using-dfs/
    /* static class Graph{ 
          
        int vertices; 
        // Linked list for adjacency list of a vertex 
        LinkedList<Integer> adjacencyList []; 
  
        @SuppressWarnings("unchecked") 
        public Graph(int vertices) 
        { 
            this.vertices = vertices; 
            adjacencyList = new LinkedList[vertices]; 
            
            for (int i = 0; i<vertices ; i++)  
            { 
                adjacencyList[i] = new LinkedList<>(); 
            } 
        } 
          
        // Function for adding edges 
        public void addEdge(int source, int dest) 
        { 
            adjacencyList.addFirst(dest); 
            adjacencyList[dest].addFirst(source); 
        } 
    } 
  
    // Function to check if the graph is connected or not 
    public void isConnected(Graph graph){ 
  
        int vertices = graph.vertices; 
        LinkedList<Integer> adjacencyList [] = graph.adjacencyList; 
  
        // Take a boolean visited array 
        boolean[] visited = new boolean[vertices]; 
  
        // Start the DFS from vertex 0 
        DFS(0, adjacencyList, visited); 
  
        // Check if all the vertices are visited 
        // Set connected to False if one node is unvisited 
        boolean connected = true; 
        
        for (int i = 0; i <visited.length ; i++) { 
            if(!visited[i]){ 
                connected = false; 
                break; 
            } 
        } 
        
        if(connected){ 
            System.out.println("Graph is connected"); 
        }else{ 
            System.out.println("Graph is disconnected"); 
        } 
    } 
  
    public void DFS(int source, LinkedList<Integer> adjacencyList [], boolean[] visited){ 
  
        // Mark the vertex visited as True 
        visited = true; 
  
        // Travel the adjacent neighbours 
        for (int i = 0; i <adjacencyList.size() ; i++) { 
            
            int neighbour = adjacencyList.get(i); 
            
            if(visited[neighbour]==false){ 
                
                // Call DFS from neighbour 
                DFS(neighbour, adjacencyList, visited); 
            } 
        } 
    //Implementar funcion que elimina vertice
    }*/
    
    //Algoritmo para el segundo caso 
    //https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/?ref=lbp
    //O prims algorithm
    
    
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
