package org.team100.frc2026.targeting;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;

import org.team100.lib.targeting.FiringParameters;
import org.team100.lib.util.InterpolatingMap100;
import org.team100.lib.util.OptUtil;
import org.team100.lib.util.Relation;

/**
 * Interpolates shooting parameters, given range in meters.
 */
public class ShooterTable {
    private static final boolean DEBUG = false;

    private final Relation<FiringParameters> m_table;
    private final DoubleFunction<Double> m_rangeToSpeed;
    private final DoubleFunction<Double> m_rangeToElevation;
    private final DoubleFunction<Double> m_rangeToTof;
    private final InterpolatingMap100<Double, FiringParameters> m_rangeMap;

    /**
     * Returns empty for out-of-range, so if you want to be sure to return
     * something, include "far away" points in the list.
     */
    public ShooterTable(List<FiringParameters> rows) {
        m_table = new Relation<>();
        m_table.addAll(rows);
        m_rangeToSpeed = m_table.function(
                FiringParameters::range, FiringParameters::speed);
        m_rangeToElevation = m_table.function(
                FiringParameters::range, FiringParameters::elevation);
        m_rangeToTof = m_table.function(
                FiringParameters::range, FiringParameters::tof);
        m_rangeMap = m_table.map(FiringParameters::range,
                FiringParameters::interpolate);
    }

    /**
     * Hood angle for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     */
    public OptionalDouble angle(double rangeM) {
        return OptUtil.emptyIfNull(m_rangeToElevation.apply(rangeM));
    }

    /**
     * Drum speed for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     */
    public OptionalDouble speed(double rangeM) {
        return OptUtil.emptyIfNull(m_rangeToSpeed.apply(rangeM));
    }

    /**
     * Time of flight for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     */
    public OptionalDouble tof(double rangeM) {
        return OptUtil.emptyIfNull(m_rangeToTof.apply(rangeM));
    }

    /**
     * Firing parameters for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     */
    public Optional<FiringParameters> forRange(double rangeM) {
        FiringParameters v = m_rangeMap.get(rangeM);
        if (v == null) {
            if (DEBUG) {
                System.out.printf("no result for range %f min %f max %f\n",
                        rangeM, m_rangeMap.firstKey(), m_rangeMap.lastKey());
            }
            return Optional.empty();
        }
        return Optional.of(v);
    }
}