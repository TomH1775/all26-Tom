package org.team100.lib.subsystems.discus.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusMech;
import org.team100.lib.visualization.ArmVisualization;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SetupMech implements Runnable {
    private final double CONTROL_SCALE = 0.1;
    private final DiscusMech m_discus;
    private final ArmVisualization m_viz;

    public SetupMech() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        XboxController controller = new XboxController(0);

        m_discus = new DiscusMech(logger, currentLog);
        m_viz = new ArmVisualization(m_discus::getPosition, "discus", 0);
        m_discus.setDefaultCommand(m_discus.position(
                () -> CONTROL_SCALE * controller.getLeftX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // button 1, "z" in the sim
        new Trigger(controller::getAButton).whileTrue(m_discus.home());
        // button 2, "x" in the sim
        new Trigger(controller::getBButton).onTrue(m_discus.zero());
    }

    @Override
    public void run() {
        m_viz.run();
    }
}
