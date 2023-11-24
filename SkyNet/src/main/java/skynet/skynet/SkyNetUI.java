
package skynet.skynet;

import Graph.CustomGraph;
import Graph.Vertex;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.algorithms.layout.Layout;
import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;
import org.jgrapht.graph.DefaultEdge;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.SparseGraph;
import edu.uci.ics.jung.graph.SparseMultigraph;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.VisualizationViewer;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import java.util.Set;
import javax.swing.JFrame;
import javax.swing.JOptionPane;



/**
 *
 * @author pavel
 */
public class SkyNetUI extends javax.swing.JFrame {
    CustomGraph graph = new CustomGraph(this);
    public SkyNetUI() {
        initComponents();
        ScrollPathList.setVisible(false);
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jPanel1 = new javax.swing.JPanel();
        jPanel2 = new javax.swing.JPanel();
        BtnLoadMap = new javax.swing.JButton();
        BtnRestrictGoods = new javax.swing.JButton();
        btnDisposeSimulation = new javax.swing.JButton();
        BtnDirectedWrld = new javax.swing.JButton();
        BtnPowerfulCity = new javax.swing.JButton();
        btnAnnihilateWrld = new javax.swing.JButton();
        btnChooseCityWipeOut = new javax.swing.JButton();
        btnClosestCon2Cities = new javax.swing.JButton();
        btnArmyCon2Cities = new javax.swing.JButton();
        BtnDivideWrld = new javax.swing.JButton();
        JlActualMap = new javax.swing.JLabel();
        btnMostEfficientWipeOut = new javax.swing.JButton();
        btnMostConnectedCity = new javax.swing.JButton();
        btnSaveSimulation = new javax.swing.JButton();
        ScrollPathList = new javax.swing.JScrollPane();
        PathLIst = new javax.swing.JList<>();
        JlSimulatedDestruction = new javax.swing.JLabel();
        scrollPanelActualMap = new javax.swing.JScrollPane();
        panelActualMap = new javax.swing.JPanel();
        panelSim = new javax.swing.JPanel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setMinimumSize(new java.awt.Dimension(1200, 820));
        getContentPane().setLayout(null);

        BtnLoadMap.setText("Load Map");
        BtnLoadMap.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                BtnLoadMapActionPerformed(evt);
            }
        });
        getContentPane().add(BtnLoadMap);
        BtnLoadMap.setBounds(80, 70, 130, 50);

        BtnRestrictGoods.setText("Restrict Goods");
        BtnRestrictGoods.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                BtnRestrictGoodsActionPerformed(evt);
            }
        });
        getContentPane().add(BtnRestrictGoods);
        BtnRestrictGoods.setBounds(210, 240, 130, 50);

        btnDisposeSimulation.setBackground(new java.awt.Color(102, 0, 51));
        btnDisposeSimulation.setFont(new java.awt.Font("sansserif", 0, 10)); // NOI18N
        btnDisposeSimulation.setText("Dispose Simulation");
        btnDisposeSimulation.setEnabled(false);
        btnDisposeSimulation.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnDisposeSimulationActionPerformed(evt);
            }
        });
        getContentPane().add(btnDisposeSimulation);
        btnDisposeSimulation.setBounds(660, 370, 140, 30);

        BtnDirectedWrld.setText("Directed World");
        BtnDirectedWrld.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                BtnDirectedWrldActionPerformed(evt);
            }
        });
        getContentPane().add(BtnDirectedWrld);
        BtnDirectedWrld.setBounds(710, 240, 130, 50);

        BtnPowerfulCity.setText("Wipe Out Most Powerful City");
        BtnPowerfulCity.setEnabled(false);
        BtnPowerfulCity.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                BtnPowerfulCityActionPerformed(evt);
            }
        });
        getContentPane().add(BtnPowerfulCity);
        BtnPowerfulCity.setBounds(510, 240, 190, 50);

        btnAnnihilateWrld.setText("Annihilate the World");
        btnAnnihilateWrld.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnAnnihilateWrldActionPerformed(evt);
            }
        });
        getContentPane().add(btnAnnihilateWrld);
        btnAnnihilateWrld.setBounds(1020, 240, 150, 50);

        btnChooseCityWipeOut.setText("Choose Connection to Destroy");
        btnChooseCityWipeOut.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnChooseCityWipeOutActionPerformed(evt);
            }
        });
        getContentPane().add(btnChooseCityWipeOut);
        btnChooseCityWipeOut.setBounds(740, 300, 200, 50);

        btnClosestCon2Cities.setText("Wipe Out Closest Connection/ 2 Cities");
        btnClosestCon2Cities.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnClosestCon2CitiesActionPerformed(evt);
            }
        });
        getContentPane().add(btnClosestCon2Cities);
        btnClosestCon2Cities.setBounds(250, 300, 250, 50);

        btnArmyCon2Cities.setText("Wipe Out Army Connection/ 2 Cities");
        btnArmyCon2Cities.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnArmyCon2CitiesActionPerformed(evt);
            }
        });
        getContentPane().add(btnArmyCon2Cities);
        btnArmyCon2Cities.setBounds(510, 300, 220, 50);

        BtnDivideWrld.setText("Devide World");
        BtnDivideWrld.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                BtnDivideWrldActionPerformed(evt);
            }
        });
        getContentPane().add(BtnDivideWrld);
        BtnDivideWrld.setBounds(70, 240, 130, 50);

        JlActualMap.setFont(new java.awt.Font("Source Sans Pro", 1, 24)); // NOI18N
        JlActualMap.setForeground(new java.awt.Color(255, 255, 255));
        JlActualMap.setText("Actual Map");
        getContentPane().add(JlActualMap);
        JlActualMap.setBounds(70, 370, 260, 31);

        btnMostEfficientWipeOut.setText("Most Efficient Destruction");
        btnMostEfficientWipeOut.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnMostEfficientWipeOutActionPerformed(evt);
            }
        });
        getContentPane().add(btnMostEfficientWipeOut);
        btnMostEfficientWipeOut.setBounds(840, 240, 180, 50);

        btnMostConnectedCity.setText("Most Connected City");
        btnMostConnectedCity.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnMostConnectedCityActionPerformed(evt);
            }
        });
        getContentPane().add(btnMostConnectedCity);
        btnMostConnectedCity.setBounds(350, 240, 150, 50);

        btnSaveSimulation.setBackground(new java.awt.Color(102, 0, 51));
        btnSaveSimulation.setFont(new java.awt.Font("sansserif", 0, 10)); // NOI18N
        btnSaveSimulation.setText("Save Simulation");
        btnSaveSimulation.setEnabled(false);
        btnSaveSimulation.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnSaveSimulationActionPerformed(evt);
            }
        });
        getContentPane().add(btnSaveSimulation);
        btnSaveSimulation.setBounds(1060, 370, 110, 30);

        ScrollPathList.setEnabled(false);

        PathLIst.setModel(new javax.swing.AbstractListModel<String>() {
            String[] strings = { "Item 1", "Item 2", "Item 3", "Item 4", "Item 5" };
            public int getSize() { return strings.length; }
            public String getElementAt(int i) { return strings[i]; }
        });
        ScrollPathList.setViewportView(PathLIst);

        getContentPane().add(ScrollPathList);
        ScrollPathList.setBounds(950, 20, 250, 130);

        JlSimulatedDestruction.setFont(new java.awt.Font("Source Sans Pro", 1, 24)); // NOI18N
        JlSimulatedDestruction.setForeground(new java.awt.Color(102, 0, 51));
        JlSimulatedDestruction.setText("Simulated Destruction");
        getContentPane().add(JlSimulatedDestruction);
        JlSimulatedDestruction.setBounds(810, 370, 260, 31);

        scrollPanelActualMap.setViewportView(panelActualMap);

        getContentPane().add(scrollPanelActualMap);
        scrollPanelActualMap.setBounds(50, 410, 520, 340);
        getContentPane().add(panelSim);
        panelSim.setBounds(660, 410, 510, 330);

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void BtnLoadMapActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_BtnLoadMapActionPerformed
        JFileChooser fileChooser = new JFileChooser();                
        FileNameExtensionFilter filter = new FileNameExtensionFilter("Archivos JSON", "json");
        fileChooser.setFileFilter(filter);
        

        // Mostrar el cuadro de di�logo para seleccionar un archivo
        int result = fileChooser.showOpenDialog(this);

        // Comprobar si el usuario seleccion� un archivo
        if (result == JFileChooser.APPROVE_OPTION) {

            // Obtener la ruta del archivo seleccionado
            String filePath = fileChooser.getSelectedFile().getAbsolutePath();
            
            graph.loadGraph(filePath);
            graph.paintGraph(panelActualMap);
        }
    }//GEN-LAST:event_BtnLoadMapActionPerformed

    private void BtnDirectedWrldActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_BtnDirectedWrldActionPerformed
        setupButtonState();
        BtnPowerfulCity.setEnabled(true);
        graph.convertToDirectGraph();
        graph.paintGraph(panelActualMap);
    }//GEN-LAST:event_BtnDirectedWrldActionPerformed

    private void BtnPowerfulCityActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_BtnPowerfulCityActionPerformed
        setupButtonState();
    }//GEN-LAST:event_BtnPowerfulCityActionPerformed

    private void btnArmyCon2CitiesActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnArmyCon2CitiesActionPerformed
        setupButtonState();
        Set<String> availableCities = graph.getVertexNames();
        String city1 = showInputDialog("Enter the name of the first city interested in:",availableCities);
        String city2 = showInputDialog("Enter the name of the second city interested in:",availableCities);
    }//GEN-LAST:event_btnArmyCon2CitiesActionPerformed

    private void btnDisposeSimulationActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnDisposeSimulationActionPerformed
        int dialogResult = JOptionPane.showConfirmDialog(this, "Are you sure you want to dispose the simulation?", "Dispose Simulation", JOptionPane.YES_NO_OPTION);
        if (dialogResult == JOptionPane.YES_OPTION) {
        // User clicked Yes, dispose the simulation
        panelSim.removeAll();
        enableAllButtons();
        }
    }//GEN-LAST:event_btnDisposeSimulationActionPerformed

    private void btnChooseCityWipeOutActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnChooseCityWipeOutActionPerformed
        setupButtonState();
    }//GEN-LAST:event_btnChooseCityWipeOutActionPerformed

    private void btnSaveSimulationActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnSaveSimulationActionPerformed
        int dialogResult = JOptionPane.showConfirmDialog(this, "Are you sure you want to save the simulation?", "Save Simulation", JOptionPane.YES_NO_OPTION);
        if (dialogResult == JOptionPane.YES_OPTION) {
        // User clicked Yes, save the simulation
        enableAllButtons();
        }
    }//GEN-LAST:event_btnSaveSimulationActionPerformed

    private void BtnDivideWrldActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_BtnDivideWrldActionPerformed
        setupButtonState();
    }//GEN-LAST:event_BtnDivideWrldActionPerformed

    private void BtnRestrictGoodsActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_BtnRestrictGoodsActionPerformed
        setupButtonState();
    }//GEN-LAST:event_BtnRestrictGoodsActionPerformed

    private void btnMostConnectedCityActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnMostConnectedCityActionPerformed
        setupButtonState();
    }//GEN-LAST:event_btnMostConnectedCityActionPerformed

    private void btnMostEfficientWipeOutActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnMostEfficientWipeOutActionPerformed
        setupButtonState();
    }//GEN-LAST:event_btnMostEfficientWipeOutActionPerformed

    private void btnAnnihilateWrldActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnAnnihilateWrldActionPerformed
        setupButtonState();
    }//GEN-LAST:event_btnAnnihilateWrldActionPerformed

    private void btnClosestCon2CitiesActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnClosestCon2CitiesActionPerformed
        setupButtonState();
        Set<String> availableCities = graph.getVertexNames();
        String city1 = showInputDialog("Enter the name of the first city interested in:",availableCities);
        String city2 = showInputDialog("Enter the name of the second city interested in:",availableCities);
    }//GEN-LAST:event_btnClosestCon2CitiesActionPerformed
    
    private String showInputDialog(String message,Set<String> availableCities) {
        String userInput = null;
        while (userInput == null || !availableCities.contains(userInput)) {
        userInput = JOptionPane.showInputDialog(this, message);
        if (userInput == null) {
            // The user clicked cancel, handle it as needed
            break;
        } else if (!availableCities.contains(userInput)) {
            JOptionPane.showMessageDialog(this, "Invalid city name. Please select a valid city.", "Invalid Input", JOptionPane.ERROR_MESSAGE);
        }
    }
    return userInput;
    }
    public void AnnihilationPosible(){
        JOptionPane.showMessageDialog(null, "Annihilation is possible!", "Annihilation Status", JOptionPane.INFORMATION_MESSAGE);
    }
    public void AnnihilationNotPosibleNotEven(){
        JOptionPane.showMessageDialog(null, "Annihilation is not possible. In-degree and out-degree are not equal for each vertex.", "Annihilation Status", JOptionPane.WARNING_MESSAGE);
    }
    public void AnnihilationNotPosibleNotConnected(){
        JOptionPane.showMessageDialog(null, "Annihilation is not possible. The graph is not connected.", "Annihilation Status", JOptionPane.WARNING_MESSAGE);
    }    private void setupButtonState() {
        // Enable Save and Dispose buttons, and disable others
        btnSaveSimulation.setEnabled(true);
        btnDisposeSimulation.setEnabled(true);

        BtnLoadMap.setEnabled(false);
        BtnRestrictGoods.setEnabled(false);
        BtnDirectedWrld.setEnabled(false);
        BtnPowerfulCity.setEnabled(false);
        btnAnnihilateWrld.setEnabled(false);
        btnChooseCityWipeOut.setEnabled(false);
        btnClosestCon2Cities.setEnabled(false);
        btnArmyCon2Cities.setEnabled(false);
        BtnDivideWrld.setEnabled(false);
        btnMostEfficientWipeOut.setEnabled(false);
        btnMostConnectedCity.setEnabled(false);
    }
     private void enableAllButtons() {
        btnSaveSimulation.setEnabled(false);
        btnDisposeSimulation.setEnabled(false);
        // Enable all resting buttons
        BtnLoadMap.setEnabled(true);
        BtnRestrictGoods.setEnabled(true);
        BtnDirectedWrld.setEnabled(true);
        btnAnnihilateWrld.setEnabled(true);
        btnChooseCityWipeOut.setEnabled(true);
        btnClosestCon2Cities.setEnabled(true);
        btnArmyCon2Cities.setEnabled(true);
        BtnDivideWrld.setEnabled(true);
        btnMostEfficientWipeOut.setEnabled(true);
        btnMostConnectedCity.setEnabled(true);
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(SkyNetUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(SkyNetUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(SkyNetUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(SkyNetUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new SkyNetUI().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton BtnDirectedWrld;
    private javax.swing.JButton BtnDivideWrld;
    private javax.swing.JButton BtnLoadMap;
    private javax.swing.JButton BtnPowerfulCity;
    private javax.swing.JButton BtnRestrictGoods;
    private javax.swing.JLabel JlActualMap;
    private javax.swing.JLabel JlSimulatedDestruction;
    private javax.swing.JList<String> PathLIst;
    private javax.swing.JScrollPane ScrollPathList;
    private javax.swing.JButton btnAnnihilateWrld;
    private javax.swing.JButton btnArmyCon2Cities;
    private javax.swing.JButton btnChooseCityWipeOut;
    private javax.swing.JButton btnClosestCon2Cities;
    private javax.swing.JButton btnDisposeSimulation;
    private javax.swing.JButton btnMostConnectedCity;
    private javax.swing.JButton btnMostEfficientWipeOut;
    private javax.swing.JButton btnSaveSimulation;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel panelActualMap;
    private javax.swing.JPanel panelSim;
    private javax.swing.JScrollPane scrollPanelActualMap;
    // End of variables declaration//GEN-END:variables
}
