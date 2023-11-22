package skynet.skynet;

import Graph.Vertex;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.SparseMultigraph;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import org.jgrapht.graph.DefaultEdge;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

public class X {

    public X() {

    }

    public void run(JPanel panel) {
        // Create a JUNG graph
        Graph<String, String> jungGraph = new SparseMultigraph<>();

        // Add vertices to the JUNG graph
        Vertex vertex1 = new Vertex("Vertex1", 10, 5, 3);
        Vertex vertex2 = new Vertex("Vertex2", 8, 3, 2);
        Vertex vertex3 = new Vertex("Vertex3", 15, 7, 5);

        jungGraph.addVertex(vertex1.getVertex());
        jungGraph.addVertex(vertex2.getVertex());
        jungGraph.addVertex(vertex3.getVertex());

        // Add edges to the JUNG graph with labels
        jungGraph.addEdge("1", vertex1.getVertex(), vertex2.getVertex());
        jungGraph.addEdge("2", vertex2.getVertex(), vertex3.getVertex());
        jungGraph.addEdge("3", vertex3.getVertex(), vertex2.getVertex());

        // Create JUNG visualization
        BasicVisualizationServer<String, String> vv = new BasicVisualizationServer<>(new CircleLayout<>(jungGraph));
        vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller());

        // Set edge labels to be displayed
        vv.getRenderContext().setEdgeLabelTransformer(edge -> "Edge " + edge);

        // Display the graph in a JFrame
        panel.removeAll();
        panel.add(vv);
        panel.revalidate();
        panel.repaint();
    }
}
