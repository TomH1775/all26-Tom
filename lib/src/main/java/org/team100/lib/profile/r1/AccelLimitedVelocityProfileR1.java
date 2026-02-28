package org.team100.lib.profile.r1;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

/**
 * Acceleration and velocity limits applied in velocity space.
 * 
 * The domain objects are abused here: "x" is velocity, "v" is acceleration.
 */
public class AccelLimitedVelocityProfileR1 implements ProfileR1 {

    private final double m_maxA;
    private final double m_maxD;

    public AccelLimitedVelocityProfileR1(double maxA, double maxD) {
        m_maxA = maxA;
        m_maxD = maxD;
    }

    @Override
    public ControlR1 calculate(double dt, ControlR1 setpoint, ModelR1 goal) {
        double goalV = goal.x();
        double setpointV = setpoint.x();
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
        return new ControlR1(setpoint.x() + dv, dv / dt);
    }

    @Override
    public ProfileR1 scale(double s) {
        throw new UnsupportedOperationException("Unimplemented method 'scale'");
    }

}
