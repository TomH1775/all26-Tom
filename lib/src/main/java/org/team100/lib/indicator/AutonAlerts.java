package org.team100.lib.indicator;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.visualization.AutonVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public class AutonAlerts {
    private static final String START_MISSING = "Starting position MISSING!!";
    private final Supplier<AnnotatedCommand> m_autons;
    private final Supplier<Pose2d> m_robotPose;
    private final Consumer<Pose2d> m_poseSetter;
    private final Alert m_noStartingPosition;
    private final Alert m_startingPositionOk;
    private final Alert m_mismatchedAlliance;
    private final AutonVisualization m_viz;

    public AutonAlerts(
            LoggerFactory fieldLogger,
            Supplier<AnnotatedCommand> autons,
            Alerts alerts,
            Supplier<Pose2d> robotPose,
            Consumer<Pose2d> poseSetter) {
        m_autons = autons;
        m_robotPose = robotPose;
        m_poseSetter = poseSetter;
        m_noStartingPosition = alerts.add(START_MISSING, AlertType.kWarning);
        m_startingPositionOk = alerts.add("Starting position OK", AlertType.kInfo);
        m_mismatchedAlliance = alerts.add("Wrong Alliance!", AlertType.kWarning);
        m_viz = new AutonVisualization(fieldLogger);
    }

    public void run() {
        // checkStart();
        // checkAlliance();
        viz();
    }

    public void clear() {
        m_viz.clear();
    }

    private void viz() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        m_viz.show(cmd);
    }

    private void checkStart() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Pose2d start = cmd.start();
        if (start == null) {
            m_noStartingPosition.setText(START_MISSING);
            start(false);
        } else {
            if (RobotBase.isReal()
                    || Experiments.instance.enabled(Experiment.TestAutonAlert)) {
                // for a real robot, we complain if we're far from the correct place.
                Pose2d pose = m_robotPose.get();
                DeltaSE2 delta = DeltaSE2.delta(start, pose);
                if (delta.l2Norm() < 0.2) {
                    // good enough
                    start(true);
                } else {
                    m_noStartingPosition.setText(
                            String.format("Starting position error %s", delta));
                    start(false);
                }
            } else {
                // in simulation we force the position so it's always ok
                m_poseSetter.accept(start);
                start(true);
            }
        }
    }

    private void start(boolean ok) {
        m_noStartingPosition.set(!ok);
        m_startingPositionOk.set(ok);
    }

    private void checkAlliance() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Alliance alliance = cmd.alliance();
        if (alliance == null) {
            // works for either
            return;
        }
        Alliance dsAlliance = DriverStation.getAlliance().orElse(null);
        if (dsAlliance == null) {
            // not set yet
            return;
        }
        if (alliance != dsAlliance) {
            m_mismatchedAlliance.set(true);
        } else {
            m_mismatchedAlliance.set(false);
        }
    }

}
