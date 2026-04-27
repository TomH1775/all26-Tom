package org.team100.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

public interface Integrator<States extends Num, Inputs extends Num> {

    /**
     * @param f  dynamics
     * @param x  initial state
     * @param u  control input
     * @param dt time step (s)
     * @return new state
     */
    Matrix<States, N1> integrate(
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u,
            double dt);

}