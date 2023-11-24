/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package skynet.skynet;

/**
 *
 * @author fabri
 */
public class WeightEdge {
    private double weight1;
    private double weight2;
    private double weight3;

    public WeightEdge(double weight1, double weight2, double weight3) {
        this.weight1 = weight1;
        this.weight2 = weight2;
        this.weight3 = weight3;
    }

    public void setWeight1(double weight1) {
        this.weight1 = weight1;
    }

    public void setWeight2(double weight2) {
        this.weight2 = weight2;
    }

    public void setWeight3(double weight3) {
        this.weight3 = weight3;
    }

    public double getWeight1() {
        return weight1;
    }

    public double getWeight2() {
        return weight2;
    }

    public double getWeight3() {
        return weight3;
    }

    
}
