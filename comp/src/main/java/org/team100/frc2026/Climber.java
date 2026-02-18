package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.util.CanId;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.NeutralMode100;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Climber extends SubsystemBase {
    private final BareMotor m_motor;
    private final OnboardAngularPositionServo m_servo;
    private static final double m_level0 = 0.1;
    private static final double m_level1 = 90;
    private static final double m_level3 = 180;

    public Climber(LoggerFactory parent, CanId CanId) {
        LoggerFactory log = parent.type(this);

        switch (Identity.instance) {

            case COMP_BOT -> {
                m_motor = new KrakenX60Motor(
                        log,
                        new CanId(0),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        60, 1,
                        new SimpleDynamics(log, 0, 0),
                        new Friction(log, 0, 0, 0, 0),
                        new PIDConstants(log, 0, 0, 0, 0, 0, 0));
                IncrementalBareEncoder encoder = m_motor.encoder();

                TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(log, 1, 2, 0.05);

                ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
                double initialPosition = 0;
                double gearRatio = 2;
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }

            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
                IncrementalProfile profile = new TrapezoidIncrementalProfile(log, 1, 2, 0.05);
                ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, ()-> profile, 0.05, 0.05);
                PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
                IncrementalBareEncoder encoder = m_motor.encoder();

                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
                RotaryMechanism climberMech = new RotaryMechanism(
                        log,
                        m_motor,
                        sensor,
                        1,
                        Double.NEGATIVE_INFINITY,
                        Double.POSITIVE_INFINITY);
                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }
        }
    }

    public Command setClimb0() {
        return runOnce(this::setL0);
    }

    public Command setClimb1() {
        return runOnce(this::setL1);
    }

    public Command setClimb3() { 
        return runOnce(this::setL3);
    }
    
    private void setL0() {
        m_servo.setPositionProfiled(m_level0, 0);
    }

    private void setL1() {
        m_servo.setPositionProfiled(m_level1, 0);
    }

    private void setL3() {
        m_servo.setPositionProfiled(m_level3, 0);
    }
        @Override
    public void periodic() {
        m_servo.periodic();
    }
}
