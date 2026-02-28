package org.team100.lib.profile.r1;

import org.team100.lib.state.VelocityControlR1;

/**
 * Velocity control only, for things like flywheels where position is
 * unimportant.
 */
public interface VelocityProfileR1 {
    /**
     * Return the control for dt in the future.
     */
    VelocityControlR1 calculate(double dt, VelocityControlR1 setpoint, double goal);
}
