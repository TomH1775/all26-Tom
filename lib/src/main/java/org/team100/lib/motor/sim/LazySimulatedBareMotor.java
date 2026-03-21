package org.team100.lib.motor.sim;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.sim.SimulatedBareEncoder;

/** A simulated motor that runs for awhile, and then stops. */
public class LazySimulatedBareMotor implements BareMotor {
    private final LoggerFactory m_log;
    private final BareMotor m_delegate;
    private final double m_timeout;
    private double m_startTime;
    private boolean m_running;

    public LazySimulatedBareMotor(LoggerFactory parent, BareMotor delegate, double timeout) {
        m_log = parent.type(this);
        m_delegate = delegate;
        m_timeout = timeout;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_delegate.setTorqueLimit(torqueNm);
    }

    @Override
    public void setDutyCycle(double output) {
        if (output < 1e-3) {
            m_running = false;
            m_delegate.setDutyCycle(output);
        } else if (m_running) {
            if (getTime() > m_timeout) {
                m_running = false;
                m_delegate.setDutyCycle(0);
            } else {
                m_delegate.setDutyCycle(output);
            }
        } else {
            m_running = true;
            resetTimer();
            m_delegate.setDutyCycle(output);
        }
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        if (velocityRad_S < 1e-3) {
            m_running = false;
            m_delegate.setVelocity(velocityRad_S, accelRad_S2, torqueNm);
        } else if (m_running) {
            if (getTime() > m_timeout) {
                m_running = false;
                m_delegate.setVelocity(0, 0, 0);
            } else {
                m_delegate.setVelocity(velocityRad_S, accelRad_S2, torqueNm);
            }
        } else {
            m_running = true;
            resetTimer();
            m_delegate.setVelocity(velocityRad_S, accelRad_S2, torqueNm);
        }
    }

    @Override
    public double getVelocityRad_S() {
        return m_delegate.getVelocityRad_S();
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_delegate.getUnwrappedPositionRad();
    }

    @Override
    public double getCurrent() {
        // running means low current
        if (m_running)
            return 10;
        // not running because the torque (thus current) required is higher
        return 100;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        m_delegate.setUnwrappedEncoderPositionRad(positionRad);
    }

    @Override
    public void setUnwrappedPosition(double positionRad, double velocityRad_S, double accelRad_S2, double torqueNm) {
        m_delegate.setUnwrappedPosition(positionRad, velocityRad_S, accelRad_S2, torqueNm);
    }

    @Override
    public double kROhms() {
        return m_delegate.kROhms();
    }

    @Override
    public double kTNm_amp() {
        return m_delegate.kTNm_amp();
    }

    @Override
    public double kFreeSpeedRPM() {
        return m_delegate.kFreeSpeedRPM();
    }

    @Override
    public IncrementalBareEncoder encoder() {
        return new SimulatedBareEncoder(m_log, this);
    }

    @Override
    public void stop() {
        m_running = false;
        m_delegate.stop();
    }

    @Override
    public void reset() {
        m_delegate.reset();
    }

    @Override
    public void close() {
        m_delegate.close();
    }

    @Override
    public void periodic() {
        m_delegate.periodic();
    }

    @Override
    public void play(double freq) {
        m_delegate.play(freq);
    }

    @Override
    public double getSupplyCurrent() {
        return m_delegate.getSupplyCurrent();
    }

    private double getTime() {
        return Takt.get() - m_startTime;
    }

    private void resetTimer() {
        m_startTime = Takt.get();
    }
}
