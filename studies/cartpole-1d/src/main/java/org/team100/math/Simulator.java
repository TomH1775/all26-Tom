package org.team100.math;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

/**
 * Simulate a system with a control law.
 */
public class Simulator<States extends Num, Inputs extends Num> {
    private final Function<Matrix<States, N1>, Matrix<Inputs, N1>> control;
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f;
    private final Integrator<States, Inputs> integrator;
    private final double dt;
    private final int steps;
    private final List<Double> t;
    private final List<Matrix<States, N1>> states;

    public Simulator(
            double dt,
            double durationSecs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Integrator<States, Inputs> integrator,
            Function<Matrix<States, N1>, Matrix<Inputs, N1>> control) {
        this.control = control;
        this.dt = dt;
        this.f = f;
        this.integrator = integrator;
        steps = (int) (durationSecs / dt);
        t = new ArrayList<Double>();
        states = new ArrayList<Matrix<States, N1>>();
    }

    public void run(Vector<States> initialState) {
        double time = 0.0;
        Matrix<States, N1> state = initialState;
        for (int i = 0; i < steps; i++) {
            time += dt;
            t.add(time);
            states.add(state);

            // the target state is zero, so the error is just the state.
            Matrix<States, N1> error = state;
            Matrix<Inputs, N1> u = control.apply(error);
            state = integrator.integrate(f, state, u, dt);
        }
    }

    public double[] series(int i) {
        return states.stream().mapToDouble(x -> x.get(i, 0)).toArray();
    }

    public double[] t() {
        return t.stream().mapToDouble(Double::doubleValue).toArray();
    }
}
