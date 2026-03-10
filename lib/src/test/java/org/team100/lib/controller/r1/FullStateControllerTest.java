package org.team100.lib.controller.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.tuning.Mutable;

class FullStateControllerTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZero() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, false, 0.01, 0.01);
        double u = c.calculate(new ModelR1(0, 0), new ModelR1(0, 0));
        assertEquals(0, u, DELTA);
    }

    @Test
    void testK1() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, false, 0.01, 0.01);
        double u = c.calculate(new ModelR1(0, 0), new ModelR1(1, 0));
        assertEquals(4, u, DELTA);
    }

    @Test
    void testK1b() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, false, 0.01, 0.01);
        double u = c.calculate(new ModelR1(1, 0), new ModelR1(0, 0));
        assertEquals(-4, u, DELTA);
    }

    @Test
    void testKangle() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, true, 0.01, 0.01);
        // at -3, near pi, goal is 3, across pi
        double u = c.calculate(new ModelR1(-3, 0), new ModelR1(3, 0));
        // the correct course is reverse
        assertEquals(-1.133, u, DELTA);
    }

    @Test
    void testK2() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, false, 0.01, 0.01);
        double u = c.calculate(new ModelR1(0, 0), new ModelR1(0, 1));
        // feedforward = reference velocity, but there's no feedforward
        // feedback = velocity error * k2
        assertEquals(0.25, u, DELTA);
    }

    @Test
    void testK2b() {
        Mutable.unpublishAll();
        FullStateFeedback c = new FullStateFeedback(logger, 4, 0.25, false, 0.01, 0.01);
        double u = c.calculate(new ModelR1(0, 1), new ModelR1(0, 0));
        // slow down
        assertEquals(-0.25, u, DELTA);
    }
}
