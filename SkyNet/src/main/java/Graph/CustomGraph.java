package Graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.graph.SparseGraph;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import edu.uci.ics.jung.visualization.renderers.Renderer;
import java.awt.Dimension;
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
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.GraphWalk;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.jgrapht.graph.SimpleGraph;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.traverse.BreadthFirstIterator;
import skynet.skynet.SkyNetUI;
/**
 *
 * @author fabri
 */
public class CustomGraph {
    final private String FILEPATH = "src/main/java/Data/graph.json";
    private org.jgrapht.Graph<Vertex, DefaultEdge> graph = new SimpleWeightedGraph<>(DefaultEdge.class);
    private org.jgrapht.Graph<Vertex, DefaultEdge> directedGraph = new DefaultDirectedWeightedGraph<>(DefaultEdge.class);
    
    private org.jgrapht.Graph<Vertex, DefaultEdge> simulatedGraph = new SimpleWeightedGraph<>(DefaultEdge.class);
    private org.jgrapht.Graph<Vertex, DefaultEdge> simulatedDirectedGraph = new DefaultDirectedWeightedGraph<>(DefaultEdge.class);
    
    private boolean isDirected;
    public SkyNetUI window;

    public CustomGraph() {
    }
    
    public CustomGraph(SkyNetUI window){
        this.window=window;
    }
    
    public void loadGraph(String route){
        this.isDirected = false;
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
    
    public void paintGraph(String command, JPanel panel){
        
        if (!isDirected) {
            edu.uci.ics.jung.graph.Graph<String, String> jungGraph = new SparseGraph<>();
            if (command.equals("real")) {
                jungGraph = convertJGraphTtoJUNG(graph);
            } 
            if (command.equals("simulated")) {
                jungGraph = convertJGraphTtoJUNG(simulatedGraph);
            }
            // Create JUNG visualization
            BasicVisualizationServer<String, String> vv = new BasicVisualizationServer<>(new CircleLayout<>(jungGraph));
            vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller());

            // Set edge labels to be displayed
            vv.getRenderContext().setEdgeLabelTransformer(edge -> edge);

            // Display the graph in a JFrame
            panel.removeAll();
            panel.add(vv);
            panel.revalidate();
            panel.repaint();

            // Add the graph visualization to the panel
            panel.add(vv);
        }
        if (isDirected) {
            edu.uci.ics.jung.graph.Graph<String, String> jungGraph =  new DirectedSparseGraph<>();
            if (command.equals("real")) {
                jungGraph = convertJGraphTDirectedtoJUNG(directedGraph);
            } 
            if (command.equals("simulated")) {
                jungGraph = convertJGraphTDirectedtoJUNG( simulatedDirectedGraph);
            }
            // Create JUNG visualization
            // Layout for the graph
            CircleLayout<String, String> layout = new CircleLayout<>(jungGraph);
            //layout.setSize(new Dimension(300, 300));

            // Visualization server
            BasicVisualizationServer<String, String> vv = new BasicVisualizationServer<>(layout);
            ///vv.setPreferredSize(new Dimension(350, 350));

            // Set up vertex and edge renderers
            // Set up vertex and edge renderers
            vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller());
            vv.getRenderContext().setEdgeLabelTransformer(new ToStringLabeller());
            vv.getRenderer().getVertexLabelRenderer().setPosition(Renderer.VertexLabel.Position.CNTR);

            // Display the graph in a JFrame
            panel.removeAll();
            panel.add(vv);
            panel.revalidate();
            panel.repaint();

            // Add the graph visualization to the panel
            panel.add(vv);
        }
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
                String edgeIdentifier = source.getVertex()  + " - " +   target.getVertex();
                jungGraph.addEdge(edgeIdentifier, source.getVertex(), target.getVertex());
            }
        }

        return jungGraph;
    }
    
    private static edu.uci.ics.jung.graph.Graph<String, String> convertJGraphTDirectedtoJUNG(org.jgrapht.Graph<Vertex, DefaultEdge> jGraphTGraph) {
        edu.uci.ics.jung.graph.Graph<String, String> jungGraph = new DirectedSparseGraph<>();

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
                String edgeIdentifier = source.getVertex()  + " - " + target.getVertex() ;
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
    
    public Set<String> getVertexNames() {
        Set<String> vertexNames = new HashSet<>();
        for (Vertex vertex : graph.vertexSet()) {
            vertexNames.add(vertex.getVertex()); 
        }
        return vertexNames;
    }
    // -------------------------------------------- Disconnect graph (Case 1) ----------------------------------------
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
    

    // -------------------------------------------- Minimum spanning tree by goods (Case 2) ----------------------------------------
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

        // Annul the corresponding edges in the original graph
        for (DefaultEdge edge : minimumSpanningTree.edgeSet()) {
            graph.removeEdge(edge);
        }

        // Visualize the graph after annulment (You can replace this with your own visualization logic)
        
    }
    
    private void visualizeGraph(Graph<Vertex, DefaultEdge> graphToVisualize) {
        System.out.println("Visualizing Graph:");
        for (DefaultEdge edge : graphToVisualize.edgeSet()) {
            Vertex source = graphToVisualize.getEdgeSource(edge);
            Vertex target = graphToVisualize.getEdgeTarget(edge);
            System.out.println(source + " -- " + target + " Goods: " + ((Edge) edge).getGoods());
        }
    }

    
    
    // -------------------------------------------- Convert to directed graph (Case 3) ----------------------------------------
    public static <V, E> boolean isConnected(Graph<V, E> graph) {
        ConnectivityInspector<V, E> inspector = new ConnectivityInspector<>(graph);
        return inspector.isConnected();
    }
    
    public void convertToDirectGraph(){
        simulatedDirectedGraph = convertToDirectedWeightedGraph(graph);
        this.isDirected = true;
    }
    
    private static <V, E> DefaultDirectedWeightedGraph<V, DefaultEdge> convertToDirectedWeightedGraph(
        Graph<V, E> sourceGraph) {
        DefaultDirectedWeightedGraph<V, DefaultEdge> directedWeightedGraph =
                new DefaultDirectedWeightedGraph<>(DefaultEdge.class);

        // Map to store mapping between original vertices and corresponding vertices in the new graph
        Map<V, V> vertexMapping = new HashMap<>();

        // Copy vertices
        for (V vertex : sourceGraph.vertexSet()) {
            V newVertex = vertex;
            directedWeightedGraph.addVertex(newVertex);
            vertexMapping.put(vertex, newVertex);
        }

        // Copy edges and their weights
        for (E edge : sourceGraph.edgeSet()) {
            V sourceVertex = sourceGraph.getEdgeSource(edge);
            V targetVertex = sourceGraph.getEdgeTarget(edge);
            double weight = sourceGraph.getEdgeWeight(edge);

            V newSourceVertex = vertexMapping.get(sourceVertex);
            V newTargetVertex = vertexMapping.get(targetVertex);

            DefaultEdge directedEdge = directedWeightedGraph.addEdge(newSourceVertex, newTargetVertex);
            directedWeightedGraph.setEdgeWeight(directedEdge, weight);
        }

        return directedWeightedGraph;
    }

    
    public void saveSimulation(){
        this.graph = this.simulatedGraph;
        this.directedGraph = this.simulatedDirectedGraph;
    }
    public void deleteSimulation(){
        this.graph = null;
        this.directedGraph = null;
    }
    
    // -------------------------------------------- Most Potent Military Node (Case 4) ----------------------------------------
    //Djistra https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/?ref=lbp
    /*public void findAndRemoveMostPotentMilitaryNode() {
        // Find the node with the highest military potential
        Vertex mostPotentMilitaryNode = findMostPotentMilitaryNode();
        
        if (mostPotentMilitaryNode != null) {
            // Determine all paths from any node to the most potent military node
            List<GraphPath<Vertex, DefaultEdge>> efficientPaths = determineEfficientPathsToNode(mostPotentMilitaryNode);

            // List and remove the efficient paths
            listAndRemovePaths(efficientPaths);

            // Visualize the updated graph
            
        } else {
            System.out.println("No nodes found in the graph.");
        }
    }

    private Vertex findMostPotentMilitaryNode() {
        return graph.vertexSet().stream()
                .max(Comparator.comparingDouble(vertex -> ((Vertex) vertex).getMilitaryPotential()))
                .orElse(null);
    }
 
    private List<GraphPath<Vertex, DefaultEdge>> determineEfficientPathsToNode(Vertex targetNode) {
        List<GraphPath<Vertex, DefaultEdge>> efficientPaths = new ArrayList<>();
        
        BreadthFirstIterator<Vertex, DefaultEdge> iterator = new BreadthFirstIterator<>(graph, targetNode);
        while (iterator.hasNext()) {
            Vertex sourceNode = iterator.next();
            if (!sourceNode.equals(targetNode)) {
                DijkstraShortestPath<Vertex, DefaultEdge> dijkstra =
                        new DijkstraShortestPath<>(graph, sourceNode, v -> graph.getEdgeWeight(v));

                GraphPath<Vertex, DefaultEdge> shortestPath = dijkstra.getPath(targetNode);
                if (shortestPath != null) {
                    efficientPaths.add(shortestPath);
                }
            }
        }

        return efficientPaths;
    }

    private void listAndRemovePaths(List<GraphPath<Vertex, DefaultEdge>> pathsToRemove) {
        System.out.println("Efficient Paths to Most Potent Military Node:");
        for (GraphPath<Vertex, DefaultEdge> path : pathsToRemove) {
            System.out.println("Path: " + path.getVertexList() +
                    " Distance: " + path.getWeight() +
                    " Military Power: " + calculateTotalMilitaryPower(path));

            // Remove the path from the graph
            for (DefaultEdge edge : path.getEdgeList()) {
                graph.removeEdge(edge);
            }
        }
    }

    private double calculateTotalMilitaryPower(GraphPath<Vertex, DefaultEdge> path) {
        return path.getEdgeList().stream()
                .mapToDouble(edge -> ((Edge) edge).getMilitary())
                .sum();
    }
    */
    
    // -------------------------------------------- Euler Circuit (Case 5) ----------------------------------------
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

    
    // -------------------------------------------- Eulerian Most Visited Vertex (Case 6) ----------------------------------------
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
        
    }
    
    // -------------------------------------------- Dijkstra Less Distance (Case 7) ----------------------------------------
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
        
    }
    */
    
    // -------------------------------------------- Dijkstra Most Military(Case 8) ----------------------------------------
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
        
    }
    */
    // -------------------------------------------- Dijkstra with All Paths (Case 9) ----------------------------------------
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

    // -------------------------------------------- Tech Level-Best Annihilation (Case 10) ----------------------------------------
    //https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/?ref=lbp
    /*    public void determineTraversalOrderAndEliminateMostExpensivePath() {
        // Determine the order of traversal based on technological level
        List<Vertex> traversalOrder = determineTraversalOrder();

        // Find the most expensive or militarily powerful path
        DefaultWeightedEdge mostExpensiveEdge = findMostExpensivePath(traversalOrder);

        if (mostExpensiveEdge != null) {
            // Remove the most expensive path from the graph
            graph.removeEdge(mostExpensiveEdge);

            // Visualize the updated graph (You can replace this with your own visualization logic)
            

            System.out.println("Most expensive path eliminated.");
        } else {
            System.out.println("No path found or the graph is empty.");
        }
    }

    private List<Vertex> determineTraversalOrder() {
        // Implement your logic to determine the order of traversal based on technological level
        // You may need to use a sorting algorithm based on the technological level attribute
        // Return the list of vertices in the desired order
        // For example, you can use Collections.sort() with a custom comparator

        List<Vertex> traversalOrder = new ArrayList<>(graph.vertexSet());
        // Sorting vertices based on technological level (Assuming you have a method getTechnologicalLevel())
        traversalOrder.sort(Comparator.comparingInt(v -> v.getTechLevel()));
        return traversalOrder;
    }

    private DefaultWeightedEdge findMostExpensivePath(List<Vertex> traversalOrder) {
        // Implement your logic to find the most expensive or militarily powerful path
        // You may use Dijkstra's algorithm or other graph traversal algorithms
        // Return the most expensive edge (path) in the graph

        DefaultWeightedEdge mostExpensiveEdge = null;
        double maxMilitaryPower = Double.MIN_VALUE;

        for (Vertex source : traversalOrder) {
            DijkstraShortestPath<Vertex, DefaultWeightedEdge> dijkstra =
                    new DijkstraShortestPath<>(graph, source, v -> graph.getEdgeWeight(v));
            
            for (Vertex target : traversalOrder) {
                if (!source.equals(target)) {
                    GraphPath<Vertex, DefaultWeightedEdge> path = dijkstra.getPath(target);
                    if (path != null) {
                        double militaryPower = calculateTotalMilitaryPower(path);
                        if (militaryPower > maxMilitaryPower) {
                            maxMilitaryPower = militaryPower;
                            mostExpensiveEdge = path.getEdgeList().get(0);
                        }
                    }
                }
            }
        }

        return mostExpensiveEdge;
    }

    private double calculateTotalMilitaryPower(GraphPath<Vertex, DefaultWeightedEdge> path) {
        // Implement your logic to calculate the total military power of a given path
        // Sum the military power of all edges in the path
        return path.getEdgeList().stream()
                .mapToDouble(edge -> ((Edge) edge).getMilitary())
                .sum();
    }
*/
}
