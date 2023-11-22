/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package skynet.skynet;

import Graph.CustomGraph;

/**
 *
 * @author fabri
 */
public class SkyNet {

    public static void main(String[] args) {
        System.out.println("Hello World!");
        
        CustomGraph graph = new CustomGraph();
        
        graph.loadGraph("src/main/java/Data/graph.json");
        
        //graph.addEdge(vertex1, vertex3, new Edge());
    }
}
