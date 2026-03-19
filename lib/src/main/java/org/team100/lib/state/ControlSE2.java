package org.team100.lib.state;

import org.team100.lib.geometry.AccelerationSE2;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectorySE2Point;
import org.team100.lib.trajectory.path.PathSE2Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Describes the state of rigid body transformations in two dimensions, the
 * SE(2) manifold (x,y,theta), where each dimension is represented by position,
 * velocity, and acceleration.
 * 
 * This could be used for navigation, or for other applications of rigid-body
 * transforms in 2d, e.g. planar mechanisms.
 * 
 * This type is used for control, which is why it includes acceleration
 * 
 * Do not try to use zero as an initial location; always initialize with the
 * current location.
 * 
 * Note: the metric used here is not the SE(2) geodesic, it treats the XY plane
 * and rotation dimensions independently.
 */
public class ControlSE2 {
    private final ControlR1 m_x;
    private final ControlR1 m_y;
    private final ControlR1 m_theta;

    public ControlSE2(ControlR1 x, ControlR1 y, ControlR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public ControlSE2(Pose2d x, VelocitySE2 v) {
        this(
                new ControlR1(x.getX(), v.x(), 0),
                new ControlR1(x.getY(), v.y(), 0),
                new ControlR1(x.getRotation().getRadians(), v.theta(), 0));
    }

    public ControlSE2(Pose2d x, VelocitySE2 v, AccelerationSE2 a) {
        this(
                new ControlR1(x.getX(), v.x(), a.x()),
                new ControlR1(x.getY(), v.y(), a.y()),
                new ControlR1(x.getRotation().getRadians(), v.theta(), a.theta()));
    }

    public ControlSE2(Pose2d x) {
        this(x, new VelocitySE2(0, 0, 0));
    }

    public ControlSE2(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    public static ControlSE2 zero() {
        return new ControlSE2(new ControlR1(), new ControlR1(), new ControlR1());
    }

    public ModelSE2 model() {
        return new ModelSE2(m_x.model(), m_y.model(), m_theta.model());
    }

    /** Component-wise difference (not geodesic) */
    public ControlSE2 minus(ControlSE2 other) {
        return new ControlSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public ControlSE2 plus(ControlSE2 other) {
        return new ControlSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(ControlSE2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), rotation());
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Rotation2d rotation() {
        return new Rotation2d(m_theta.x());
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    /** Robot-relative speeds */
    public ChassisSpeeds chassisSpeeds() {
        return SwerveKinodynamics.toInstantaneousChassisSpeeds(velocity(), rotation());
    }

    public AccelerationSE2 acceleration() {
        return new AccelerationSE2(m_x.a(), m_y.a(), m_theta.a());
    }

    public ControlR1 x() {
        return m_x;
    }

    public ControlR1 y() {
        return m_y;
    }

    public ControlR1 theta() {
        return m_theta;
    }

    /**
     * Trajectory point => ControlSE2.
     * Correctly computes centripetal acceleration.
     */
    public static ControlSE2 fromTrajectorySE2Point(TrajectorySE2Point p) {
        return ControlSE2.fromMovingPathSE2Point(p.point(), p.velocity(), p.accel());
    }

    /**
     * Point and pathwise velocity and accel => ControlSE2.
     * Correctly computes centripetal acceleration.
     */
    public static ControlSE2 fromMovingPathSE2Point(
            PathSE2Point point, double velocityM_s, double accelM_s_s) {

        WaypointSE2 waypoint = point.waypoint();
        Pose2d pose = waypoint.pose();
        DirectionSE2 direction = waypoint.course();

        double xx = pose.getTranslation().getX();
        double yx = pose.getTranslation().getY();
        double thetax = pose.getRotation().getRadians();

        Rotation2d course = direction.toRotation();
        double xv = course.getCos() * velocityM_s;
        double yv = course.getSin() * velocityM_s;
        double thetav = direction.headingRate() * velocityM_s;

        double xa = course.getCos() * accelM_s_s;
        double ya = course.getSin() * accelM_s_s;
        double thetaa = direction.headingRate() * accelM_s_s;

        // centripetal accel = v^2/r = v^2 * curvature
        // this works because the acceleration vector is always normal
        // to the course vector, and in 2d, with signed curvature, that
        // determines the vector.
        double curvRad_M = point.k();
        double centripetalAccelM_s_s = velocityM_s * velocityM_s * curvRad_M;
        double xCa = -1.0 * course.getSin() * centripetalAccelM_s_s;
        double yCa = course.getCos() * centripetalAccelM_s_s;

        return new ControlSE2(
                new ControlR1(xx, xv, xa + xCa),
                new ControlR1(yx, yv, ya + yCa),
                new ControlR1(thetax, thetav, thetaa));
    }

    public String toString() {
        return "SwerveControl(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}