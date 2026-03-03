package org.team100.frc2026;

import java.util.function.Supplier;

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
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.CanId;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {

    private final AngularPositionServo m_servo;
    private final Supplier<Translation2d> m_target;
    private final ShooterTable m_table;
    // TODO GET THIS CAN ID
    private final CanId canID = new CanId(0);

    private Supplier<ModelSE2> m_state;

    public ShooterHood(
            LoggerFactory parent,
            Supplier<ModelSE2> state,
            Supplier<Translation2d> target) {
        LoggerFactory log = parent.type(this);
        m_state = state;
        m_target = target;
        m_table = new ShooterTable();
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                float gearRatio = 10;
                PIDConstants PID = PIDConstants.makePositionPID(log, 1);
                double supplyLimit = 50;
                double statorLimit = 20;
                SimpleDynamics ff = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                KrakenX44Motor m_motor = new KrakenX44Motor(
                        log,
                        canID,
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        supplyLimit,
                        statorLimit,
                        ff,
                        friction,
                        PID);
                Talon6Encoder encoder = m_motor.encoder();

                TrapezoidProfileR1 profile = new TrapezoidProfileR1(log, 1, 2, 0.05);
                ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
                double initialPosition = 0;
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log, climberMech, ref);

            }

            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log, 600);

                ProfileR1 profile = new TrapezoidProfileR1(log, 1, 2, 0.05);
                ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
                PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

                IncrementalBareEncoder encoder = m_motor.encoder();
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, sensor, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    public Command position() {
        return startRun(this::reset, this::autoWork);
    }

    public void autoWork() {
        ModelSE2 state = m_state.get();
        Translation2d target = m_target.get();
        double rangeM = state.translation().getDistance(target);
        double angle = m_table.getAngleRad(rangeM);
        setAngle(angle);
    }

    public Command stop() {
        return run(this::stopServo).withName("Shooter Hood Stop");
    }

    public boolean onTarget() {
        return m_servo.atGoal();
    }

    /////////////////////////////////////////

    private void reset() {
        m_servo.reset();
    }

    private void stopServo() {
        m_servo.stop();
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.setPositionProfiled(value, 0);
    }

}
