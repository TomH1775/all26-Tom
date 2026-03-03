package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtend extends SubsystemBase {
    private final AngularPositionServo m_servo;

    public IntakeExtend(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(log, 8, 8, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double gearRatio = 15.3;

        switch (Identity.instance) {

            case COMP_BOT -> {
                PIDConstants PID = PIDConstants.makePositionPID(log, 1);
                double supplyLimit = 4;
                double statorLimit = 80;
                SimpleDynamics ff = new SimpleDynamics(log, 0.0, 0.0);
                Friction friction = new Friction(log, 1.26, 1.26, 0.006, 0.5);
                KrakenX44Motor m_motor = new KrakenX44Motor(
                        log, new CanId(16),
                        NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit,
                        ff, friction, PID);
                Talon6Encoder encoder = m_motor.encoder();
                double initialPosition = 0;
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log, climberMech, ref);
            }

            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log, 600);
                IncrementalBareEncoder encoder = m_motor.encoder();
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, sensor, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    // ends when complete
    public Command goToExtendedPosition() {
        return startRun(this::reset, () -> setAngle(3))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToExtendedPosition");
    }

    // never ends
    public Command goToRetractedPosition() {
        return startRun(this::reset, () -> setAngle(0))
                .withName("Intake Extend GoToRetractedPosition");
    }

    public Command stop() {
        return run(this::stopServo).withName("Intake Extend Stop");
    }

    /////////////////////////////////////////

    private void stopServo() {
        m_servo.stop();
    }

    private void reset() {
        m_servo.reset();
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.actuateWithProfile(value, 0);
    }

}
