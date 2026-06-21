package org.team100.frc2026;

import com.reduxrobotics.frames.Frame;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Demonstrate use of Redux Canandmag encoder.
 */
public class Robot extends TimedRobot {
    private final Canandmag encoder;

    public Robot() {
        encoder = new Canandmag(1);
    }

    @Override
    public void robotPeriodic() {
        double FPGATimeSec = Timer.getFPGATimestamp();

        double position = encoder.getPosition();
        SmartDashboard.putNumber("position (rotations)", position);

        double velocity = encoder.getVelocity();
        SmartDashboard.putNumber("velocity (rotations per sec)", velocity);

        Frame<Double> positionFrame = encoder.getPositionFrame();
        double framePosition = positionFrame.getValue();
        SmartDashboard.putNumber("frame position (rotations)", framePosition);

        double timestampSec = positionFrame.getTimestamp();
        SmartDashboard.putNumber("position frame timestamp (sec)", timestampSec);

        double lagSec = FPGATimeSec - timestampSec;
        SmartDashboard.putNumber("lag (sec)", lagSec);
    }
}
