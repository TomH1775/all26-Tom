package org.team100.frc2026.targeting;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.targeting.FiringParameters;

import edu.wpi.first.math.geometry.Translation2d;

/** Chooses where to aim, supplies parameters. */
public class Targeter {
    private final Supplier<Translation2d> m_position;
    /** High shots to score */
    private final ShooterTable m_score;
    /** Low shots to save power */
    private final ShooterTable m_lob;

    public Targeter(Supplier<Translation2d> position) {
        m_position = position;
        // TODO: TUNE
        m_score = new ShooterTable(
                List.of(

                        new FiringParameters(1.65, 12, 0, 0.8),
                        new FiringParameters(2.28, 14, 0, 0.96),
                        new FiringParameters(2.99, 14, 0.15, 0.845),
                        new FiringParameters(3.52, 15, 0.2, 0.912),
                        new FiringParameters(4.3, 17, 0.35, 0.84),
                        new FiringParameters(5.32, 19, 0.3, 1.03)));

        // TODO: TUNE
        m_lob = new ShooterTable(
                List.of(
                        new FiringParameters(6, 17, 0.25, 1.268),
                        new FiringParameters(16, 17, 0.25, 1.268)));
    }

    /**
     * Firing parameters for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     */
    public Optional<FiringParameters> forRange(double rangeM) {
        Translation2d p = m_position.get();
        if (FieldConstants2026.isInAllianceZone(p)) {
            return m_score.forRange(rangeM);
        }
        if (FieldConstants2026.isInNeutralZone(p)) {
            return m_lob.forRange(rangeM);
        }
        return Optional.empty();
    }

}
