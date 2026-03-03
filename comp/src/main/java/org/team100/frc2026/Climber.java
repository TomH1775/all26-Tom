package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final BareMotor m_motor;
    private final BareMotor m_motor2;
    private final AngularPositionServo m_servo;
    private final AngularPositionServo m_servo2;
    private static final double m_level0 = 0;
    private static final double m_level1 = -Math.PI / 2;
    private static final double m_level3 = -Math.PI;

    public Climber(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("motor1");
        LoggerFactory log2 = log.name("motor2");
        ProfileR1 profile = new TrapezoidProfileR1(log, 3, 5, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double gearRatio = 28;
        double initialPosition = 0;

        switch (Identity.instance) {
            case COMP_BOT, TEST_BOARD_B0 -> {
                int supplyLimit = 60;
                int statorLimit = 40;
                SimpleDynamics ff = new SimpleDynamics(log, 0, 0);
                Friction friction = new Friction(log, 0, 0, 0, 0);
                PIDConstants pid = new PIDConstants(log, 1, 0, 0, 0, 0, 0);
                m_motor = new KrakenX60Motor(
                        log1,
                        new CanId(18),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        ff,
                        friction,
                        pid);
                IncrementalBareEncoder encoder = m_motor.encoder();
                RotaryMechanism climberMech = new RotaryMechanism(
                        log1, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log1, climberMech, ref);
                m_motor2 = new KrakenX60Motor(
                        log2,
                        new CanId(19),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        ff,
                        friction,
                        pid);
                IncrementalBareEncoder encoder2 = m_motor2.encoder();
                RotaryMechanism climberMech2 = new RotaryMechanism(
                        log2, m_motor2, encoder2, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo2 = new OutboardAngularPositionServo(log2, climberMech2, ref);
            }

            default -> {
                m_motor = new SimulatedBareMotor(log1, 600);
                IncrementalBareEncoder encoder = m_motor.encoder();
                RotaryMechanism climberMech = new RotaryMechanism(
                        log1, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_motor2 = new SimulatedBareMotor(log2, 600);
                IncrementalBareEncoder encoder2 = m_motor2.encoder();
                RotaryMechanism climberMech2 = new RotaryMechanism(
                        log2, m_motor2, encoder2, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log1, climberMech, ref);
                m_servo2 = new OutboardAngularPositionServo(log2, climberMech2, ref);
            }
        }
    }

    public Command setClimb0() {
        return new FunctionalCommand(
                this::reset,
                this::setL0,
                x -> {
                },
                () -> false,
                this);
    }

    public Command setClimb1() {
        return new FunctionalCommand(
                this::reset,
                this::setL1,
                x -> {
                },
                () -> false,
                this);
    }

    public Command setClimb3() {
        return new FunctionalCommand(
                this::reset,
                this::setL3,
                x -> {
                },
                () -> false,
                this);
    }

    private void reset() {
        m_servo.reset();
        m_servo2.reset();
    }

    private void setL0() {
        m_servo.actuateWithProfile(m_level0, 0);
        m_servo2.actuateWithProfile(m_level0, 0);
    }

    private void setL1() {
        m_servo.actuateWithProfile(m_level1, 0);
        m_servo2.actuateWithProfile(m_level1, 0);
    }

    private void setL3() {
        m_servo.actuateWithProfile(m_level3, 0);
        m_servo2.actuateWithProfile(m_level3, 0);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
        m_servo2.periodic();
    }
}
