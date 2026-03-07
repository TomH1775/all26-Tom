package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Solution for infinite muzzle velocity, for testing.
 * 
 * Azimuth is always the bearing.
 * Azimuth velocity is the apparent target motion.
 * Elevation is always zero.
 */
public class LaserSolver implements Solver {

    private final DoubleFunction<FiringParameters> m_rangeToParams;

    public LaserSolver(DoubleFunction<FiringParameters> rangeToParams) {
        m_rangeToParams = rangeToParams;
    }

    @Override
    public Optional<Solution> solve(
            ModelSE2 state,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        final Translation2d robotPosition = state.translation();

        // Target relative to robot
        final Translation2d T0 = targetPosition.minus(robotPosition);
        double rangeM = T0.getNorm();

        FiringParameters params = m_rangeToParams.apply(rangeM);

        // Does not include target velocity in solution velocity.
        Rotation2d absoluteBearing = TargetUtil.absoluteBearing(
                robotPosition, targetPosition);
        double azimuthVelocity = TargetUtil.targetMotion(
                state, targetPosition);
        return Optional.of(new Solution(
                absoluteBearing,
                azimuthVelocity,
                new Rotation2d(params.elevation())));
    }

}
