package org.team100.lib.trajectory.constraint;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A constant velocity limit within a rectangle; no limit outside.
 */
public class VelocityLimitRegionConstraint implements TimingConstraint {
    private final Rectangle2d m_region;
    private final Mutable m_maxV;

    public VelocityLimitRegionConstraint(LoggerFactory parent, Rectangle2d region, double maxV) {
        if (maxV < 0)
            throw new IllegalArgumentException();
        LoggerFactory log = parent.type(this);
        m_region = region;
        m_maxV = new Mutable(log, "maxV", maxV);
    }

    @Override
    public double maxV(PathSE2Point point) {
        final Translation2d translation = point.waypoint().pose().getTranslation();
        if (m_region.contains(translation))
            return m_maxV.getAsDouble();
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocity) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return Double.NEGATIVE_INFINITY;
    }

}