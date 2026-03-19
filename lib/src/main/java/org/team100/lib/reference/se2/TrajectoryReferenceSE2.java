package org.team100.lib.reference.se2;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelSE2Logger;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.trajectory.TrajectorySE2Entry;
import org.team100.lib.trajectory.TrajectorySE2Point;
import org.team100.lib.trajectory.TrajectorySE2;

/** Produces references based on a trajectory. */
public class TrajectoryReferenceSE2 implements ReferenceSE2 {
    private final LoggerFactory m_log;
    private final TrajectorySE2 m_trajectory;
    private final ModelSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final BooleanLogger m_log_done;
    private final ModelSE2Logger m_log_goal;
    private final DoubleLogger m_log_progress;
    private double m_startTimeS;

    public TrajectoryReferenceSE2(
            LoggerFactory parent,
            TrajectorySE2 trajectory) {
        m_log = parent.type(this);
        m_trajectory = trajectory;
        m_log_progress = m_log.doubleLogger(Level.TRACE, "progress");
        m_log_current = m_log.modelSE2Logger(Level.TRACE, "current");
        m_log_next = m_log.controlSE2Logger(Level.TRACE, "next");
        m_log_done = m_log.booleanLogger(Level.TRACE, "done");
        m_log_goal = m_log.modelSE2Logger(Level.TRACE, "goal");
    }

    /** Ignores the measurement, resets the trajectory timer. */
    @Override
    public void initialize(ModelSE2 measurement) {
        m_startTimeS = Takt.get();
    }

    @Override
    public ModelSE2 current() {
        ModelSE2 current = sample(progress()).model();
        m_log_current.log(() -> current);
        return current;
    }

    @Override
    public ControlSE2 next() {
        ControlSE2 next = sample(progress() + TimedRobot100.LOOP_PERIOD_S);
        m_log_next.log(() -> next);
        return next;
    }

    @Override
    public boolean done() {
        boolean done = m_trajectory.isDone(progress());
        m_log_done.log(() -> done);
        return done;
    }

    @Override
    public ModelSE2 goal() {
        TrajectorySE2Entry lastPoint = m_trajectory.getLastPoint();
        ModelSE2 goal = ControlSE2.fromMovingPathSE2Point(
                lastPoint.point().point(), lastPoint.point().velocity(), lastPoint.point().accel()).model();
        m_log_goal.log(() -> goal);
        return goal;
    }

    ////////////////////////////////////////////////////

    private double progress() {
        double progress = Takt.get() - m_startTimeS;
        m_log_progress.log(() -> progress);
        return progress;
    }

    private ControlSE2 sample(double t) {
        TrajectorySE2Entry sample = m_trajectory.sample(t);
        TrajectorySE2Point point = sample.point();
        return ControlSE2.fromMovingPathSE2Point(
                point.point(),
                point.velocity(),
                point.accel());
    }
}
