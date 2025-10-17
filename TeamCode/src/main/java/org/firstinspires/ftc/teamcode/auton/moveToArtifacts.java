package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Object + Distance Detection (12.7cm ≈ 5in)", group = "Vision")
public class moveToArtifacts extends LinearOpMode {

    private static final double REAL_DIAMETER = 12.7; // cm (≈ 5 inches)
    OpenCvWebcam webcam;
    ColorCircleDistancePipeline pipeline;

    @Override
    public void runOpMode() {
        int camViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewId);

        pipeline = new ColorCircleDistancePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.log().add("Camera open error: " + errorCode);
            }
        });

        // Wait until a detection appears before start
        while (!isStarted() && !isStopRequested()) {
            ColorCircleDistancePipeline.DetectedCircle green = pipeline.getLastGreen();
            ColorCircleDistancePipeline.DetectedCircle purple = pipeline.getLastPurple();

            if (green != null) {
                pipeline.calibrate(30.0, green); // Known distance = 30cm
                telemetry.addLine("✅ Calibrated using green ball!");
            } else if (purple != null) {
                pipeline.calibrate(30.0, purple);
                telemetry.addLine("✅ Calibrated using purple ball!");
            } else {
                telemetry.addLine("No ball detected yet...");
            }
            telemetry.update();
        }

        waitForStart();

        // Auto-calibrate using whichever color is visible first
        double knownDistance = 50.0; // known distance in cm
        ColorCircleDistancePipeline.DetectedCircle purple = pipeline.getLastPurple();
        ColorCircleDistancePipeline.DetectedCircle green = pipeline.getLastGreen();
        if (purple != null) pipeline.calibrate(knownDistance, purple);
        else if (green != null) pipeline.calibrate(knownDistance, green);
        telemetry.addLine("Calibrated at 50 cm. Move balls to test distance.");
        telemetry.update();

        while (opModeIsActive()) {
            purple = pipeline.getLastPurple();
            green = pipeline.getLastGreen();

            Double distPurpleCm = pipeline.getPurpleDistance();
            Double distGreenCm = pipeline.getGreenDistance();

            telemetry.addLine("=== Object Distance ===");

            if (purple != null && distPurpleCm != null) {
                double distPurpleIn = distPurpleCm ;
                telemetry.addLine("🟣 Purple detected");
                telemetry.addData("Center (x,y)", "(%.1f, %.1f)", purple.center.x, purple.center.y);
                telemetry.addData("Radius (px)", "%.1f", purple.radius);
                telemetry.addData("Distance", "%.1f cm (%.1f in)", distPurpleCm, distPurpleIn);
            }

            if (green != null && distGreenCm != null) {
                double distGreenIn = distGreenCm ;
                telemetry.addLine("🟢 Green detected");
                telemetry.addData("Center (x,y)", "(%.1f, %.1f)", green.center.x, green.center.y);
                telemetry.addData("Radius (px)", "%.1f", green.radius);
                telemetry.addData("Distance", "%.1f cm (%.1f in)", distGreenCm, distGreenIn);
            }

            if (purple == null && green == null) {
                telemetry.addLine("No objects detected");
            }

            telemetry.update();
        }

        webcam.stopStreaming();
    }

    // ---------------------------------------------------------
    // OpenCV pipeline
    // ---------------------------------------------------------
    public static class ColorCircleDistancePipeline extends org.openftc.easyopencv.OpenCvPipeline {

        private Mat hsv = new Mat();
        private Mat maskPurple = new Mat();
        private Mat maskGreen = new Mat();
        private Mat morphPurple = new Mat();
        private Mat morphGreen = new Mat();
        private Mat display = new Mat();

        // HSV thresholds (tune for your lighting)
        private Scalar lowPurple = new Scalar(125, 50, 50);
        private Scalar highPurple = new Scalar(160, 255, 255);
        private Scalar lowGreen = new Scalar(35, 70, 70);
        private Scalar highGreen = new Scalar(85, 255, 255);

        private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        private DetectedCircle lastPurple = null;
        private DetectedCircle lastGreen = null;

        private static final double REAL_DIAMETER = 12.7; // cm
        private double focalLength = -1;
        private boolean isCalibrated = false;

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(display);

            // Convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // Create color masks
            Core.inRange(hsv, lowPurple, highPurple, maskPurple);
            Core.inRange(hsv, lowGreen, highGreen, maskGreen);

            // Morphological cleanup
            Imgproc.morphologyEx(maskPurple, morphPurple, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskPurple, morphPurple, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(maskGreen, morphGreen, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskGreen, morphGreen, Imgproc.MORPH_CLOSE, kernel);

            // Detect circles
            lastPurple = detectBestCircleFromMask(morphPurple);
            lastGreen = detectBestCircleFromMask(morphGreen);

            // Draw
            if (lastPurple != null) {
                Imgproc.circle(display, lastPurple.center, (int) lastPurple.radius, new Scalar(255, 0, 255), 3);
                Imgproc.circle(display, lastPurple.center, 3, new Scalar(255, 0, 0), -1);
            }

            if (lastGreen != null) {
                Imgproc.circle(display, lastGreen.center, (int) lastGreen.radius, new Scalar(0, 255, 0), 3);
                Imgproc.circle(display, lastGreen.center, 3, new Scalar(0, 0, 255), -1);
            }

            return display;
        }

        private DetectedCircle detectBestCircleFromMask(Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            DetectedCircle best = null;
            double bestScore = 0;

            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area < 1000) continue; // ignore small blobs

                MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(c2f, center, radius);

                double circleArea = Math.PI * radius[0] * radius[0];
                double ratio = area / circleArea;

                if (ratio > bestScore && ratio > 0.6) {
                    bestScore = ratio;
                    best = new DetectedCircle(center, radius[0]);
                }
            }
            return best;
        }

        /** Calibrate focal length using known distance (cm) */
        public void calibrate(double knownDistance, DetectedCircle detection) {
            if (detection == null) return;
            double pixelDiameter = 2.0 * detection.radius;
            if (pixelDiameter <= 0) return;
            focalLength = (pixelDiameter * knownDistance) / REAL_DIAMETER;
            isCalibrated = true;
        }

        private Double estimateDistance(DetectedCircle c) {
            if (!isCalibrated || c == null) return null;
            double pixelDiameter = 2.0 * c.radius;
            if (pixelDiameter <= 0) return null;
            return ((REAL_DIAMETER * focalLength) / pixelDiameter)/2.54; // distance in cm
        }

        public Double getPurpleDistance() {
            return estimateDistance(lastPurple);
        }

        public Double getGreenDistance() {
            return estimateDistance(lastGreen);
        }

        public DetectedCircle getLastPurple() {
            return lastPurple;
        }

        public DetectedCircle getLastGreen() {
            return lastGreen;
        }

        public static class DetectedCircle {
            public final Point center;
            public final float radius;

            public DetectedCircle(Point c, float r) {
                center = c;
                radius = r;
            }
        }
    }
}