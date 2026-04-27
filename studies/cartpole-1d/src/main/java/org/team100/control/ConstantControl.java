package org.team100.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** Always returns the same value. */
public class ConstantControl<States extends Num, Inputs extends Num>
        implements ControlLaw<States, Inputs> {
    private final Matrix<Inputs, N1> u;

    public ConstantControl(Matrix<Inputs, N1> u) {
        this.u = u;
    }

    @Override
    public Matrix<Inputs, N1> f(Matrix<States, N1> x) {
        return u;
    }

}
