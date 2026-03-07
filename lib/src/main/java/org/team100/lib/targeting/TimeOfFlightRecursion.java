package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Time-of-flight recursion, as described by @oblarg is an iterative approach
 * that works as follows:
 * 
 * 1. shoot at the target location
 * 2. evolve the target location based on the TOF of that shot
 * 3. repeat until the shooting solution doesn't change
 * 
 * see
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815/21?u=truher
 */
public class TimeOfFlightRecursion implements Solver {
    private static final boolean DEBUG = false;

    /** Look up solution parameters for range. */
    private final DoubleFunction<FiringParameters> m_rangeToParams;
    /** Solution TOF tolerance, seconds. */
    private final double m_tolerance;

    /**
     * @param rangeToParams FiringParameters as a function of desired range
     * @param tolerance     complete when the solution doesn't change more than
     *                      this, in seconds
     */
    public TimeOfFlightRecursion(
            DoubleFunction<FiringParameters> rangeToParams,
            double tolerance) {
        m_rangeToParams = rangeToParams;
        m_tolerance = tolerance;
    }

    /** This is a separate class to make it easier to test. */
    static class Looper {
        static record LoopSolution(
                FiringParameters params, Translation2d targetPositionAtTOF) {
        }

        private final DoubleFunction<FiringParameters> m_rangeToParams;
        /** Target position relative to robot. */
        private final Translation2d m_T0;
        /** Target velocity relative to robot. */
        private final GlobalVelocityR2 m_vT;

        public Looper(
                DoubleFunction<FiringParameters> rangeToParams,
                Translation2d T0,
                GlobalVelocityR2 vT) {
            m_rangeToParams = rangeToParams;
            m_T0 = T0;
            m_vT = vT;
        }

        LoopSolution step(Double targetTOF) {

            // where is the target at the specified TOF?
            Translation2d targetPositionAtTOF = m_vT.integrate(m_T0, targetTOF);
            double rangeAtTOF = targetPositionAtTOF.getNorm();

            // What gun elevation gets to that range, and what is the
            // ball TOF to get there?
            FiringParameters params = m_rangeToParams.apply(rangeAtTOF);
            LoopSolution var2 = new LoopSolution(
                    params, targetPositionAtTOF);
            if (DEBUG)
                System.out.printf("range %f elevation %f tof %f\n",
                        var2.targetPositionAtTOF.getNorm(),
                        var2.params.elevation(), var2.params.tof());
            return var2;
        }
    }

    @Override
    public Optional<Solution> solve(
            ModelSE2 state,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        final Translation2d robotPosition = state.translation();
        final GlobalVelocityR2 robotVelocity = state.velocityR2();

        // Target relative to robot
        final Translation2d T0 = targetPosition.minus(robotPosition);
        double rangeM = T0.getNorm();
        // Target velocity relative to robot
        final GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);
        
        Looper looper = new Looper(m_rangeToParams, T0, vT);

        // Initial guess is the initial location.
        double targetTOF = m_rangeToParams.apply(rangeM).tof();

        for (int i = 0; i < 100; ++i) {
            Looper.LoopSolution soln = looper.step(targetTOF);
            double ballTOF = soln.params.tof();
            if (MathUtil.isNear(targetTOF, ballTOF, m_tolerance)) {
                // Found a good solution!
                double targetMotion = TargetUtil.targetMotion(
                        state, soln.targetPositionAtTOF);
                return Optional.of(new Solution(
                        soln.targetPositionAtTOF.getAngle(),
                        targetMotion,
                        new Rotation2d(soln.params.elevation())));
            }
            // try again with a new target TOF guess
            targetTOF = ballTOF;
        }
        return Optional.empty();
    }

}
