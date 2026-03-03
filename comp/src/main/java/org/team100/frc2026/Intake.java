package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final BareMotor m_motor;
    private final BareMotor m_motor2;

    public Intake(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);

        LoggerFactory log1 = log.name("motor1");
        LoggerFactory log2 = log.name("motor2");
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 50;
                SimpleDynamics ff = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                m_motor = new KrakenX44Motor(
                        log1, new CanId(15),
                        NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit,
                        ff, friction, PID);

                m_motor2 = new KrakenX44Motor(
                        log2, new CanId(17),
                        NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit,
                        ff, friction, PID);

            }

            default -> {
                m_motor = new SimulatedBareMotor(log1, 600);
                m_motor2 = new SimulatedBareMotor(log2, 600);
            }
        }
    }

    public Command intake() {
        return run(this::intakeFullSpeed).withName("Intake Full Speed");
    }

    public Command stop() {
        return run(this::stopMotor).withName("Intake Stop");
    }

    @Override
    public void periodic() {
        m_motor.periodic();
        m_motor2.periodic();
    }

    ////////////////////////////////

    private void stopMotor() {
        m_motor.stop();
        m_motor2.stop();
    }

    private void intakeFullSpeed() {
        // motor max velocity is 6000 RPM or 100 rev/s or 600 rad/s
        // we want to choose about 75% of that, so 450 rad/s
        double velocityRad_S = 450;
        m_motor.setVelocity(velocityRad_S, 0, 0);
        m_motor2.setVelocity(velocityRad_S, 0, 0);
        // m_motor.setDutyCycle(1);
        // System.out.println(BumpZones.BLUE_BUMP_LEFT);
    }

}
