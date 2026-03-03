package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SerializerUpper extends SubsystemBase {
    public static final CanId canID1 = new CanId(8);
    public static final CanId canID2 = new CanId(9);
    private static final double WHEEL_DIAMETER_M = 0.1;
    private static final double TOLERANCE_M_S = 1.0;
    private static final double GEAR_RATIO = 1.0;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    private final Shooter m_Shooter;

    private final double m_speed = 30;

    public SerializerUpper(LoggerFactory parent, Shooter shooter) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("SerializerUpper1");
        LoggerFactory log2 = log.name("SerializerUpper2");

        m_Shooter = shooter;
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, WHEEL_DIAMETER_M);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 40;

                SimpleDynamics dynamics = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                // TODO: set canIDs
                BareMotor m_motor1 = new KrakenX44Motor(
                        log1, canID1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                BareMotor m_motor2 = new KrakenX44Motor(
                        log2, canID2, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                // verify these numbers
                LinearMechanism mechanism1 = new LinearMechanism(
                        log1, m_motor1, m_motor1.encoder(), GEAR_RATIO, WHEEL_DIAMETER_M,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                LinearMechanism mechanism2 = new LinearMechanism(
                        log2, m_motor2, m_motor2.encoder(), GEAR_RATIO, WHEEL_DIAMETER_M,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism1, ref, TOLERANCE_M_S);

                m_servo2 = new OutboardLinearVelocityServo(
                        log2, mechanism2, ref, TOLERANCE_M_S);
            }
            default -> {
                SimulatedBareMotor m_motor1 = new SimulatedBareMotor(log1, 600);
                SimulatedBareMotor m_motor2 = new SimulatedBareMotor(log2, 600);

                LinearMechanism mechanism1 = new LinearMechanism(
                        log1, m_motor1, m_motor1.encoder(), GEAR_RATIO, WHEEL_DIAMETER_M,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                LinearMechanism mechanism2 = new LinearMechanism(
                        log2, m_motor2, m_motor2.encoder(), GEAR_RATIO, WHEEL_DIAMETER_M,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism1, ref, TOLERANCE_M_S);
                m_servo2 = new OutboardLinearVelocityServo(
                        log2, mechanism2, ref, TOLERANCE_M_S);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    public Command serializerUpperFullspeed() {
        return run(this::fullSpeed).withName(" to Shooter full speed");
    }

    public Command testSerializerUpper() {
        return run(this::testSpeed).withName("test to Shooter full speed");
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop Shooter feed");
    }

    public void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void fullSpeed() {
        double Velocity = 5.0;
        double Zero = 0;
        if (m_Shooter.atSpeed()) {
            m_servo1.setVelocityProfiled(Velocity);
            m_servo2.setVelocityProfiled(Velocity);
        } else {
            m_servo1.setVelocityProfiled(Zero);
            m_servo2.setVelocityProfiled(Zero);
        }
    }

    private void testSpeed() {
        double Velocity = 1;
        m_servo1.setDutyCycle(Velocity);
        m_servo2.setDutyCycle(Velocity);
    }


    public void setSpeed(double Velocity) {
        m_servo1.setVelocityProfiled(Velocity);
        m_servo2.setVelocityProfiled(Velocity);
    }

    public void setSerializerSpeed() {
        setSpeed(m_speed);
    }

}
