/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package Graph;

import java.util.ArrayList;

/**
 *
 * @author fabri
 */
public class Vertex {
    private String vertex;
    private int soldiers;
    private int missiles;
    private int techLevel;
    private ArrayList<Edge> edges;
    
    public Vertex(){
        this.vertex = "";
        this.soldiers = -1;
        this.missiles = -1;
        this.techLevel = -1;
        this.edges = new ArrayList<Edge>();
    }
    
    public Vertex(String vertex, int soldiers, int missiles, int techLevel){
        this.vertex = vertex;
        this.soldiers = soldiers;
        this.missiles = missiles;
        this.techLevel = techLevel;
        this.edges = new ArrayList<Edge>();
    }

    public String getVertex() {
        return vertex;
    }

    public int getSoldiers() {
        return soldiers;
    }

    public int getMissiles() {
        return missiles;
    }

    public int getTechLevel() {
        return techLevel;
    }
    public double getMilitaryPotential(){
        return soldiers + missiles;
    }

    public ArrayList<Edge> getEdges() {
        return edges;
    }
}
