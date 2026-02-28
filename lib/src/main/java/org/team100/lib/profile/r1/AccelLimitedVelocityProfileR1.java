package org.team100.lib.profile.r1;

import org.team100.lib.state.VelocityControlR1;

/**
 * Acceleration and velocity limits applied in velocity space.
 */
public class AccelLimitedVelocityProfileR1 implements VelocityProfileR1 {

    private final double m_maxA;
    private final double m_maxD;

    public AccelLimitedVelocityProfileR1(double maxA, double maxD) {
        m_maxA = maxA;
        m_maxD = maxD;
    }

    public AccelLimitedVelocityProfileR1(double maxA) {
        this(maxA, maxA);
    }

    @Override
    public VelocityControlR1 calculate(double dt, VelocityControlR1 setpoint, double goal) {
        double goalV = goal;
        double setpointV = setpoint.v();
        double dv = goalV - setpointV;

        if (setpointV >= 0) {
            if (goalV > setpointV) {
                // accel
                dv = Math.min(dv, dt * m_maxA);
            } else {
                // decel
                dv = Math.max(dv, -dt * m_maxD);
            }
        } else {
            if (goalV < setpointV) {
                // accel (in the negative direction)
                dv = Math.max(dv, -dt * m_maxA);
            } else {
                // decel (in the positive direction)
                dv = Math.min(dv, dt * m_maxD);
            }
        }
        return new VelocityControlR1(setpoint.v() + dv, dv / dt);
    }
}
