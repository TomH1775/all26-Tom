package org.team100.lib.motor;

import org.team100.lib.config.Friction;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

public class MockBareMotor implements BareMotor, IncrementalBareEncoder {
    public double output = 0;
    /** rad */
    public double position = 0;
    /** rad/s */
    public double velocity = 0;
    /** rad/s^2 */
    public double accel = 0;
    /** Nm */
    public double torque = 0;

    /** These is for testing feedforwards. */
    public double ffVolts;
    public double frictionFFVolts;
    public double backEMFVolts;
    public double torqueFFVolts;
    public double accelFFVolts;
    private final SimpleDynamics m_ff;
    private final Friction m_friction;

    public MockBareMotor(SimpleDynamics ff, Friction friction) {
        m_ff = ff;
        m_friction = friction;
    }

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVelocity(double motorRad_S, double motorRad_S2, double torqueNm) {
        velocity = motorRad_S;
        accel = motorRad_S2;
        torque = torqueNm;
    }

    @Override
    public void setUnwrappedPosition(
            double motorRad, double motorRad_S, double motorRad_S2, double torqueNm) {
        position = motorRad;
        velocity = motorRad_S;
        accel = motorRad_S2;
        torque = torqueNm;

        frictionFFVolts = m_friction.frictionFFVolts(motorRad_S);
        backEMFVolts = backEMFVoltage(motorRad_S);
        torqueFFVolts = getTorqueFFVolts(torqueNm);
        accelFFVolts = m_ff.accelFFVolts(motorRad_S, motorRad_S2);
        ffVolts = backEMFVolts + frictionFFVolts + torqueFFVolts + accelFFVolts;
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 6000;
    }

    @Override
    public IncrementalBareEncoder encoder() {
        return this;
    }

    @Override
    public void stop() {
        this.output = 0;
        this.velocity = 0;
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return this.velocity;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return this.position;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        this.position = positionRad;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

}
