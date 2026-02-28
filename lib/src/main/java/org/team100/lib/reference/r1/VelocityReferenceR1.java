package org.team100.lib.reference.r1;

import org.team100.lib.state.VelocityControlR1;

/**
 * Velocity control only, for things like flywheels where position is
 * unimportant.
 */
public interface VelocityReferenceR1 {
    void setGoal(double goal);

    /** Set setpoint to measurement. */
    void init(double measurement);

    VelocityControlR1 get();

    /** The profile has reached the goal. */
    boolean profileDone();
}
