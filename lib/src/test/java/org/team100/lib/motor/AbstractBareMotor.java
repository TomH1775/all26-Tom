package org.team100.lib.motor;

import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

public class AbstractBareMotor implements BareMotor {

    @Override
    public void play(double freq) {
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void setDutyCycle(double output) {
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
    }

    @Override
    public double getVelocityRad_S() {
        return 0;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
    }

    @Override
    public void setUnwrappedPosition(
            double positionRad, double velocityRad_S, double accelRad_S2, double torqueNm) {

    }

    @Override
    public double kROhms() {
        return 0;
    }

    @Override
    public double kTNm_amp() {
        return 0;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 0;
    }

    @Override
    public IncrementalBareEncoder encoder() {
        return null;
    }

    @Override
    public void stop() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void close() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

};