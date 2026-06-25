package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.TermCriteria;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Transform3d;

public class CornerTest {
    /**
     * One option is to ship *raw* camera corners, and for the camera to be unaware
     * of its own parameters.
     * 
     * @throws Throwable
     */
    @Test
    void testUndistortInJava() throws Throwable {
        OpenCvLoader.forceLoad();
        // from the camera
        Mat src = new Mat();
        Mat dst = new Mat();
        Mat intrinsic = new Mat();
        Mat distortion = new Mat();
        TermCriteria term = new TermCriteria(
                TermCriteria.COUNT | TermCriteria.EPS, 40, 0.01);
        Calib3d.undistortImagePoints(src, dst, intrinsic, distortion, term);
        AprilTagPoseEstimator.Config c = new AprilTagPoseEstimator.Config(
                0.1651, 100, 100, 50, 50);
        AprilTagPoseEstimator e = new AprilTagPoseEstimator(c);
        // do we have to ship this?
        double[] homography = new double[9];
        double[] corners = new double[8];
        Transform3d t = e.estimate(homography, corners);
        assertEquals(new Transform3d(), t);
    }

    /** Instead of the tag-detector solver, we can use the OpenCV one. */
    @Test
    void testSolvePNP() throws IOException {
        OpenCvLoader.forceLoad();
        MatOfPoint3f obj = new MatOfPoint3f();
        MatOfPoint2f img = new MatOfPoint2f();
        Mat intrinsic = new Mat();
        MatOfDouble distortion = new MatOfDouble();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Calib3d.solvePnP(obj, img, intrinsic, distortion, rvec, tvec, false, Calib3d.SOLVEPNP_IPPE_SQUARE);
        assertEquals(new Mat(), rvec);
        assertEquals(new Mat(), tvec);
    }
}
