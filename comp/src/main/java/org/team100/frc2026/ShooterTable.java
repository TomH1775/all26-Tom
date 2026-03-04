package org.team100.frc2026;

import org.team100.lib.util.DiscreteFunction;
import org.team100.lib.util.DiscreteFunction.Point;
import org.team100.lib.util.InterpolatingMap100;

import edu.wpi.first.math.interpolation.Interpolator;

/** Interpolates gun elevation in radians, given range in meters. */
public class ShooterTable {
    private final InterpolatingMap100<Double, Point<Double>> m_table;

    public ShooterTable() {
        m_table = loadTable();
    }

    /** Returns null for out-of-range. */
    public Double getAngleRad(double rangeM) {
        Point<Double> p = m_table.get(rangeM);
        if (p == null)
            return null;
        return p.y();
    }

    public static InterpolatingMap100<Double, Point<Double>> loadTable() {
        // x is elevation (rad)
        // y is range (m) at target height
        // value is TOF (sec)
        DiscreteFunction<Double> fn = new DiscreteFunction<>();
        fn.put(0.90, 1.49, 0.1);
        fn.put(0.78, 2.07, 0.2);
        fn.put(0.66, 2.50, 0.3);
        fn.put(0.59, 3.02, 0.4);
        fn.put(0.53, 3.59, 0.5);
        fn.put(0.48, 4.10, 0.6);
        fn.put(0.44, 4.50, 0.7);
        // x is range
        // y is elevation
        DiscreteFunction<Double> inv = fn.inverse();
        return inv.map(Interpolator.forDouble());
    }

}