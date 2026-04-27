package org.team100.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;

public class RK4Integrator<States extends Num, Inputs extends Num> implements Integrator<States, Inputs> {

    @Override
    public Matrix<States, N1> integrate(
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u,
            double dt) {
        return NumericalIntegration.rk4(f, x, u, dt);
    }
}