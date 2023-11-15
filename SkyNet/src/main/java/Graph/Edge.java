    /*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package Graph;

/**
 *
 * @author fabri
 */
public class Edge {
    private Vertex toVertex;
    private int military;
    private int goods;
    private int distance;
    
    public Edge(Vertex toVertex, int military, int goods, int distance){
        this.toVertex = toVertex;
        this.military = military;
        this.goods = goods;
        this.distance = distance;
    }

    public Vertex getToVertex() {
        return toVertex;
    }
}
