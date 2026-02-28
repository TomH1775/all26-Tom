package org.team100.lib.reference.r1;

import org.team100.lib.state.VelocityControlR1;

/** For when you won't be using the reference, i.e. only "direct" actuation. */

public class NoVelocityReferenceR1 implements VelocityReferenceR1 {

    @Override
    public void setGoal(double goal) {
    }

    @Override
    public void init(double measurement) {
    }

    @Override
    public VelocityControlR1 get() {
        return null;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

}
