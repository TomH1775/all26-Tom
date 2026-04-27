package org.team100.control;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Control input is a function of the error state.
 * 
 * @param States dimensions of error state, x
 * @param Inputs dimensions of control input u
 */
public interface ControlLaw<States extends Num, Inputs extends Num> {
    Matrix<Inputs, N1> f(Matrix<States, N1> x);
}
