package org.team100.frc2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

public class ShooterTableTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        ShooterTable t = new ShooterTable();
        assertNull(t.getAngleRad(1));
        assertEquals(0.794, t.getAngleRad(2), DELTA);
        assertEquals(0.490, t.getAngleRad(4), DELTA);
        assertNull(t.getAngleRad(6));
    }

}
