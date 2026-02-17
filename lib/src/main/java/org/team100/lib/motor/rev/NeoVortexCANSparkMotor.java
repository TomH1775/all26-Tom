package org.team100.lib.motor.rev;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Neo Vortex motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1652/
 */
public class NeoVortexCANSparkMotor extends CANSparkMotor {
    public NeoVortexCANSparkMotor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        super(parent, new SparkFlex(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, friction, pid);
    }

    /**
     * Real or simulated depending on identity.
     * 
     * PID units for outboard velocity control are duty cycle per RPM, so if you
     * want to control to a few hundred RPM, P should be something like 0.0002.
     */
    public static BareMotor get(
            LoggerFactory log, CanId can, MotorPhase phase, int statorLimit,
            SimpleDynamics ff, Friction friction, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, 600);
            default -> new NeoVortexCANSparkMotor(
                    log, can, NeutralMode100.BRAKE, phase, statorLimit, ff, friction, pid);
        };

    }

    @Override
    public double kROhms() {
        return 0.057;
    }

    @Override
    public double kTNm_amp() {
        return 0.017;
    }

    @Override
    public double kFreeSpeedRPM() {
        // this sets the back EMF voltage to zero
        // return Double.MAX_VALUE;
        return 6784;

    }

    public static SimpleDynamics ff(LoggerFactory log) {
        return new SimpleDynamics(log, 0.000, 0.000);
    }

    public static Friction friction(LoggerFactory log) {
        return new Friction(log, 0.100, 0.065, 0.0, 0.5);
    }
}
