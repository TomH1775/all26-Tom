package org.team100.lib.profile.r1;

import org.team100.lib.state.VelocityControlR1;
import org.team100.lib.util.Math100;

/**
 * Velocity limits like the way a motor works.
 * 
 * Applied in velocity space.
 */
public class CurrentLimitedExponentialVelocityProfileR1 implements VelocityProfileR1 {

    private final double m_maxV;
    private final double m_maxA;
    private final double m_maxD;
    private final double m_stallA;

    public CurrentLimitedExponentialVelocityProfileR1(double maxV, double maxA, double maxD, double stallA) {
        m_maxV = maxV;
        m_maxA = maxA;
        m_maxD = maxD;
        m_stallA = stallA;
    }

    @Override
    public VelocityControlR1 calculate(double dt, VelocityControlR1 setpoint, double goal) {
        double goalV = goal;
        double setpointV = setpoint.v();
        double dv = goalV - setpointV;

        if (setpointV >= 0) {
            if (goalV > setpointV) {
                // accel
                dv = Math.min(dv, dt * maxA(setpointV));
            } else {
                // decel
                dv = Math.max(dv, -dt * maxD());
            }
        } else {
            if (goalV < setpointV) {
                // accel (in the negative direction)
                dv = Math.max(dv, -dt * maxA(setpointV));
            } else {
                // decel (in the positive direction)
                dv = Math.min(dv, dt * maxD());
            }
        }
        return new VelocityControlR1(setpoint.v() + dv, dv / dt);
    }

    private double maxA(double setpointV) {
        double speedFraction = Math100.limit(Math.abs(setpointV) / m_maxV, 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * m_stallA;
        double currentLimitedAcceleration = m_maxA;
        return Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration);
    }

    private double maxD() {
        return m_maxD;
    }
}
