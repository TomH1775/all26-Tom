package org.team100.frc2026.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonPositions {
    
    public static final Pose2d CLIMB_LEFT = new Pose2d(1.175, 4.25, new Rotation2d(-90 * (Math.PI / 180)));
    public static final Pose2d CLIMB_RIGHT = new Pose2d(1.175, 3.1, new Rotation2d(90 * (Math.PI / 180)));

}
