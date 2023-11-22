    /*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package Graph;

import org.jgrapht.graph.DefaultEdge;

/**
 *
 * @author fabri
 */
public class Edge extends DefaultEdge{
    private String toVertex;
    private int military;
    private int goods;
    private int distance;
    
    public Edge(){
        this.toVertex = "";
        this.military = -1;
        this.goods = -1;
        this.distance = -1; 
    }
    
    public Edge(String toVertex, int military, int goods, int distance){
        this.toVertex = toVertex;
        this.military = military;
        this.goods = goods;
        this.distance = distance;
    }

    public String getToVertex() {
        return toVertex;
    }

    public int getMilitary() {
        return military;
    }

    public int getGoods() {
        return goods;
    }

    public int getDistance() {
        return distance;
    }
}
