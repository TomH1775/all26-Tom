package org.team100.frc2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterHoodTest implements Timeless2026 {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        // robot is at the origin
        // target is at the origin
        ShooterHood hood = new ShooterHood(log,
                () -> new ModelSE2(),
                () -> new Translation2d());
        // Mech starts at zero.
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Goal starts at measurement.
        assertEquals(0, hood.getUnwrappedGoal().x(), DELTA);
        Command position = hood.position();
        position.initialize();
        position.execute();
        // Out-of-bounds means there is no goal.
        assertNull(hood.getUnwrappedGoal());
        // Position has not moved
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Not on target; there is no valid target.
        assertFalse(hood.onTarget());
    }

    @Test
    void test1() {
        // robot is at the origin
        // target is at the origin
        ShooterHood hood = new ShooterHood(log,
                () -> new ModelSE2(),
                () -> new Translation2d(3, 0));
        // Mech starts at zero.
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Goal starts at measurement.
        assertEquals(0, hood.getUnwrappedGoal().x(), DELTA);
        Command position = hood.position();
        position.initialize();
        position.execute();
        // Goal is OK
        assertEquals(0.593, hood.getUnwrappedGoal().x(), DELTA);
        for (int i = 0; i < 25; ++i) {
            stepTime();
            position.execute();
            hood.periodic();
        }
        // partway there
        assertEquals(0.250, hood.getUnwrappedPositionRad(), DELTA);
        // TODO: this is taking too long, increase the profile speed
        // or use direct mode.
        for (int i = 0; i < 50; ++i) {
            stepTime();
            position.execute();
            hood.periodic();
        }
        // all the way there
        assertEquals(0.593, hood.getUnwrappedPositionRad(), DELTA);
        // finally on target
        assertTrue(hood.onTarget());
    }

}
