package org.team100.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Implements u = -Kx.
 */
public class ProportionalFeedback<States extends Num, Inputs extends Num>
        implements ControlLaw<States, Inputs> {
    private final Matrix<Inputs, States> K;

    public ProportionalFeedback(Matrix<Inputs, States> K) {
        this.K = K;
    }

    @Override
    public Matrix<Inputs, N1> f(Matrix<States, N1> x) {
        return K.times(x).times(-1);
    }

}
