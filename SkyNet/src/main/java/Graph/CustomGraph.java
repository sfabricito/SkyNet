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
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;
import java.util.stream.Collectors;
import javax.swing.JFrame;
import javax.swing.JPanel;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import org.jgrapht.Graphs;
import org.jgrapht.alg.connectivity.ConnectivityInspector;


import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.spanning.KruskalMinimumSpanningTree;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.GraphWalk;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.jgrapht.graph.SimpleGraph;
import skynet.skynet.SkyNetUI;
/**
 *
 * @author fabri
 */
public class CustomGraph {
    final private String FILEPATH = "src/main/java/Data/graph.json";
    private org.jgrapht.Graph<Vertex, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);
    public SkyNetUI window;

    public CustomGraph() {
    }
    
    public CustomGraph(SkyNetUI window){
        this.window=window;
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
    public void totalAnnihilation() {
        // Check if the graph is connected
        ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        if (!connectivityInspector.isConnected()) {
            System.out.println("The graph is not connected. Total annihilation not possible.");
            window.AnnihilationNotPosibleNotConnected();
            return;
        }

        // Check if the degrees are suitable for Eulerian circuit
        if (!checkDegreesForEulerianCircuit()) {
            System.out.println("In-degree and out-degree are not equal for each vertex. Total annihilation not possible.");
            window.AnnihilationNotPosibleNotEven();
            return;
        }

        // Find an Eulerian circuit
        List<Vertex> eulerianCircuit = findEulerianCircuit();

        // Display the Eulerian circuit
        System.out.println("Eulerian Circuit (Total Annihilation):");
        for (int i = 0; i < eulerianCircuit.size() - 1; i++) {
            Vertex source = eulerianCircuit.get(i);
            Vertex target = eulerianCircuit.get(i + 1);
            DefaultEdge edge = graph.getEdge(source, target);
            System.out.println(source + " -- " + target);
        }
        window.AnnihilationPosible();
    }

    private boolean checkDegreesForEulerianCircuit() {
        for (Vertex vertex : graph.vertexSet()) {
            int degree = graph.degreeOf(vertex);
            if (degree % 2 != 0) {
                return false;
            }
        }
        return true;
    }

    private List<Vertex> findEulerianCircuit() {
        // Create a copy of the graph to modify during the algorithm
        Graph<Vertex, DefaultEdge> copyGraph = new SimpleDirectedGraph<>(DefaultEdge.class);
        Graphs.addAllVertices(copyGraph, graph.vertexSet());

        // Instead of using Graphs.addAllEdges, manually add edges to the copyGraph
        for (DefaultEdge edge : graph.edgeSet()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            copyGraph.addEdge(source, target);
        }
        // Initialize the stack to store the circuit
        Deque<Vertex> circuitStack = new LinkedList<>();

        // Initialize the final circuit
        List<Vertex> eulerianCircuit = new ArrayList<>();

        // Select a starting vertex
        Vertex startVertex = copyGraph.vertexSet().iterator().next();
        circuitStack.push(startVertex);

        while (!circuitStack.isEmpty()) {
            Vertex currentVertex = circuitStack.peek();

            if (copyGraph.outDegreeOf(currentVertex) > 0) {
                // If there are outgoing edges, choose a neighbor and remove the edge
                Vertex nextVertex = Graphs.successorListOf(copyGraph, currentVertex).get(0);
                DefaultEdge edge = copyGraph.getEdge(currentVertex, nextVertex);
                copyGraph.removeEdge(edge);
                circuitStack.push(nextVertex);
            } else {
                // If no outgoing edges, pop the stack and add the vertex to the circuit
                circuitStack.pop();
                eulerianCircuit.add(currentVertex);
            }
        }

        // Reverse the circuit to get the correct order
        Collections.reverse(eulerianCircuit);

        return eulerianCircuit;
    }

    
    //Algoritmo para el sexto caso
    //https://www.geeksforgeeks.org/eulerian-path-and-circuit/
    //Relativo
        public void findAndRemoveMostVisitedVertices() {
        // Check if the graph is connected
        ConnectivityInspector<Vertex, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        if (!connectivityInspector.isConnected()) {
            System.out.println("The graph is not connected. Cannot find and remove most visited vertices.");
            return;
        }

        // Find an Eulerian circuit
        List<Vertex> eulerianCircuit = findEulerianCircuit();

        // Count the occurrences of each vertex in the circuit
        Map<Vertex, Integer> vertexVisitCount = new HashMap<>();
        for (Vertex vertex : eulerianCircuit) {
            vertexVisitCount.put(vertex, vertexVisitCount.getOrDefault(vertex, 0) + 1);
        }

        // Find the vertices with the maximum visit count
        int maxVisitCount = 0;
        for (int count : vertexVisitCount.values()) {
            maxVisitCount = Math.max(maxVisitCount, count);
        }
        
        final int finalMaxVisitCount = maxVisitCount;
        
        // Remove vertices with the maximum visit count
        Set<Vertex> verticesToRemove = vertexVisitCount.entrySet().stream()
                .filter(entry -> entry.getValue() == finalMaxVisitCount)
                .map(Map.Entry::getKey)
                .collect(Collectors.toSet());

        for (Vertex vertex : verticesToRemove) {
            graph.removeVertex(vertex);
        }

        // Print the Eulerian circuit and removed vertices
        System.out.println("Eulerian Circuit (Total Annihilation):");
        for (int i = 0; i < eulerianCircuit.size() - 1; i++) {
            Vertex source = eulerianCircuit.get(i);
            Vertex target = eulerianCircuit.get(i + 1);
            DefaultEdge edge = graph.getEdge(source, target);
            System.out.println(source + " -- " + target);
        }

        System.out.println("Vertices Removed:");
        verticesToRemove.forEach(System.out::println);

        // Visualize the updated graph (You can replace this with your own visualization logic)
        visualizeGraph(graph);
    }
    
    //Algoritmo para el setimo caso
    //Djistra pero yo escogiendo las ciudades
    
    // Use Dijkstra's algorithm to find the shortest path with only the "distance" attribute
    /*
    public void removeShortestPath(String startCity, String endCity) {
        DijkstraShortestPath<Vertex, DefaultEdge> dijkstra =
                new DijkstraShortestPath<>(graph, edge -> (double) ((Edge) edge).getDistance());

        SingleSourcePaths<Vertex, DefaultEdge> paths = dijkstra.getPaths(searchNodeByName(startCity));
        GraphPath<Vertex, DefaultEdge> shortestPath = paths.getPath(searchNodeByName(endCity));

        if (shortestPath == null) {
            System.out.println("No path found from " + startCity + " to " + endCity);
            return;
        }

        // Print the shortest path with distances
        System.out.println("Shortest Path from " + startCity + " to " + endCity + ":");
        for (DefaultEdge edge : shortestPath.getEdgeList()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            System.out.println(source + " -- " + target + " Distance: " + ((Edge) edge).getDistance());
        }

        // Remove the shortest path from the graph
        for (DefaultEdge edge : shortestPath.getEdgeList()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            graph.removeEdge(graph.getEdge(source, target));
        }

        // Visualize the updated graph
        System.out.println("Updated Graph:");
        visualizeGraph(graph);
    }
    */
    
    //Algoritmo para le octavo caso
    //Djistra que sume mas por poder militar
    /*
    public void removeStrongestArmyPath(String startCity, String endCity) {
        DijkstraShortestPath<Vertex, DefaultEdge> dijkstra =
                new DijkstraShortestPath<>(graph, edge -> (double) ((Edge) edge).getMilitary());

        SingleSourcePaths<Vertex, DefaultEdge> paths = dijkstra.getPaths(searchNodeByName(startCity));
        GraphPath<Vertex, DefaultEdge> StrongestArmyPath = paths.getPath(searchNodeByName(endCity));

        if (StrongestArmyPath == null) {
            System.out.println("No path found from " + startCity + " to " + endCity);
            return;
        }

        // Print the shortest path with distances
        System.out.println("Strongest Path from " + startCity + " to " + endCity + ":");
        for (DefaultEdge edge : StrongestArmyPath.getEdgeList()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            System.out.println(source + " -- " + target + " Military Strenght: " + ((Edge) edge).getMilitary());
        }

        // Remove the shortest path from the graph
        for (DefaultEdge edge : StrongestArmyPath.getEdgeList()) {
            Vertex source = graph.getEdgeSource(edge);
            Vertex target = graph.getEdgeTarget(edge);
            graph.removeEdge(graph.getEdge(source, target));
        }

        // Visualize the updated graph
        System.out.println("Updated Graph:");
        visualizeGraph(graph);
    }
    */
    
    //Algortimo para el noveno caso
    //Djistra pero que retorne todos los caminos y escoger que camino
    /*
    public void findAndRemovePaths(String startCity, String endCity) {
        // Find all paths using a modified DFS approach
        List<GraphPath<Vertex, DefaultEdge>> allPaths = findAllPaths(graph, searchNodeByName(startCity), searchNodeByName(endCity));

        if (allPaths.isEmpty()) {
            System.out.println("No path found from " + startCity + " to " + endCity);
            return;
        }

        // Print all paths and their weights
        for (int i = 0; i < allPaths.size(); i++) {
            GraphPath<Vertex, DefaultEdge> path = allPaths.get(i);
            System.out.println("Path " + (i + 1) + ": " + path.getVertexList());

            // Display weights for each characteristic (e.g., goods, distance, military)
            for (DefaultEdge edge : path.getEdgeList()) {
                Edge edgeInfo = (Edge) edge;
                System.out.println("  Goods: " + edgeInfo.getGoods() +
                        ", Distance: " + edgeInfo.getDistance() +
                        ", Military: " + edgeInfo.getMilitary());
            }

            System.out.println();
        }

        // Allow Skynet to choose and remove a path
        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter the number of the path to remove (1-" + allPaths.size() + "): ");
        int selectedPathIndex = scanner.nextInt();

        if (selectedPathIndex >= 1 && selectedPathIndex <= allPaths.size()) {
            GraphPath<Vertex, DefaultEdge> pathToRemove = allPaths.get(selectedPathIndex - 1);
            for (DefaultEdge edge : pathToRemove.getEdgeList()) {
                graph.removeEdge(edge);
            }
            System.out.println("Path removed: " + pathToRemove.getVertexList());
        } else {
            System.out.println("Invalid input. No path removed.");
        }

        // Visualize the updated graph (You can replace this with your own visualization logic)
        visualizeGraph(graph);
    }

    
    private List<GraphPath<Vertex, DefaultEdge>> findAllPaths(Graph<Vertex, DefaultEdge> graph, Vertex startVertex, Vertex endVertex) {
    List<GraphPath<Vertex, DefaultEdge>> allPaths = new ArrayList<>();
    List<Vertex> currentPath = new ArrayList<>();
    Set<Vertex> visited = new HashSet<>();

    findAllPathsDFS(graph, startVertex, endVertex, visited, currentPath, allPaths);

    return allPaths;
    }

    private void findAllPathsDFS(
            Graph<Vertex, DefaultEdge> graph,
            Vertex currentVertex,
            Vertex endVertex,
            Set<Vertex> visited,
            List<Vertex> currentPath,
            List<GraphPath<Vertex, DefaultEdge>> allPaths
    ) {
        visited.add(currentVertex);
        currentPath.add(currentVertex);

        if (currentVertex.equals(endVertex)) {
            // Found a path, add it to the list
            List<DefaultEdge> edgeList = new ArrayList<>();
            for (int i = 0; i < currentPath.size() - 1; i++) {
                edgeList.add(graph.getEdge(currentPath.get(i), currentPath.get(i + 1)));
            }
            GraphPath<Vertex, DefaultEdge> path = new GraphWalk<>(graph, currentPath, edgeList, 0.0);
            allPaths.add(path);
        } else {
            for (Vertex neighbor : Graphs.neighborSetOf(graph, currentVertex)) {
                if (!visited.contains(neighbor)) {
                    findAllPathsDFS(graph, neighbor, endVertex, visited, currentPath, allPaths);
                }
            }
        }

        visited.remove(currentVertex);
        currentPath.remove(currentPath.size() - 1);
    }*/



    
    //https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/?ref=lbp
}
