import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import edu.uci.ics.jung.visualization.renderers.Renderer;

import javax.swing.*;
import java.awt.*;

public class SkyNet {

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> createAndShowGUI());
    }

    private static void createAndShowGUI() {
        JFrame frame = new JFrame("JUNG Directed Graph Example");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 400);

        DirectedSparseGraph<String, String> graph = createSampleGraph();

        // Layout for the graph
        CircleLayout<String, String> layout = new CircleLayout<>(graph);
        layout.setSize(new Dimension(300, 300));

        // Visualization server
        BasicVisualizationServer<String, String> vv = new BasicVisualizationServer<>(layout);
        vv.setPreferredSize(new Dimension(350, 350));

        // Set up vertex and edge renderers
        // Set up vertex and edge renderers
        vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller());
        vv.getRenderContext().setEdgeLabelTransformer(new ToStringLabeller());
        vv.getRenderer().getVertexLabelRenderer().setPosition(Renderer.VertexLabel.Position.CNTR);


        frame.getContentPane().add(vv);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

    private static DirectedSparseGraph<String, String> createSampleGraph() {
        DirectedSparseGraph<String, String> graph = new DirectedSparseGraph<>();
        graph.addVertex("A");
        graph.addVertex("B");
        graph.addVertex("C");
        graph.addVertex("D");

        graph.addEdge("Edge1", "A", "B");
        graph.addEdge("Edge2", "B", "C");
        graph.addEdge("Edge3", "C", "D");
        graph.addEdge("Edge4", "D", "A");

        return graph;
    }
}
