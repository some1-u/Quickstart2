package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.*;

/**
 * FTC DECODE Autonomous
 * Uses PedroPathing for navigation and Limelight for vision.
 * Handles motif scanning, artifact collection, shooting, and parking.
 */
@Autonomous(name = "DecodeAuton_Improved", group = "Autonomous")
public class UpdatingAuton extends LinearOpMode {

    // =======================
    //  SYSTEM COMPONENTS
    // =======================
    private Follower follower;
    private Limelight3A limelight;

    // =======================
    //  GAME STATE VARIABLES
    // =======================
    private boolean motifKnown = false;
    private char[] motifOrder = {'A', 'B', 'C'};
    private int artifactIndex = 0;

    // Alliance setup
    private boolean isBlueAlliance = true;

    // =======================
    //  FIELD POSES
    // =======================
    private final Pose redShootingPose = new Pose(120, 50, 0);
    private final Pose blueShootingPose = new Pose(-120, 50, 180);
    private final Pose redParkPose = new Pose(130, 80, 0);
    private final Pose blueParkPose = new Pose(-130, 80, 180);

    // =======================
    //  SETTINGS & SAFETY
    // =======================
    private static final double ROBOT_SAFETY_DISTANCE = 18.0; // inches
    private static final double SIDESTEP_DISTANCE = 12.0;     // inches
    private static final double AUTON_TIMEOUT = 30_000;       // 30 seconds

    // Crash detection
    private Pose lastPose = new Pose(0, 0, 0);
    private long lastPoseUpdateTime = 0;
    private static final long STUCK_TIMEOUT = 2000; // 2 seconds stationary = recovery attempt

    private long startTime;

    // =======================
    //  MAIN AUTONOMOUS LOGIC
    // =======================
    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5); // artifact detection pipeline
        limelight.start();

        telemetry.addLine("🔵 Autonomous Ready — Press X for Blue, B for Red");
        telemetry.update();

        // Alliance color selector before start
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlueAlliance = true;
            if (gamepad1.b) isBlueAlliance = false;

            telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
            telemetry.update();
        }

        waitForStart();
        startTime = System.currentTimeMillis();

        // === MAIN LOOP ===
        while (opModeIsActive() && !isTimeUp()) {
            follower.update();
            detectCrashOrStuck(); // 🧠 crash recovery system

            // Step 1: Scan motif (detect order from obelisk tags)
            if (!motifKnown) {
                scanMotif();
            }

            // Step 2: Collect each artifact in correct order
            if (motifKnown && artifactIndex < motifOrder.length) {
                char target = motifOrder[artifactIndex];
                if (collectArtifactInOrder(target)) {
                    artifactIndex++;
                }
            }

            // Step 3: Shoot and park after collecting all artifacts
            if (artifactIndex >= motifOrder.length) {
                goToShootingZone();
                shootArtifacts();
                goToParkingZone();
                break; // End autonomous
            }

            idle();
        }

        // Stop all motion
        follower.breakFollowing();
    }

    // =======================
    //  SCAN MOTIF
    // =======================
    private void scanMotif() {
        LLResult res = limelight.getLatestResult();
        if (res != null && res.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    int id = f.getFiducialId();
                    if (isObeliskTag(id)) {
                        motifOrder = motifFromTagId(id);
                        motifKnown = true;

                        telemetry.addData("Motif Identified", Arrays.toString(motifOrder));
                        telemetry.update();
                        break;
                    }
                }
            }
        }
    }

    // =======================
    //  COLLECT ARTIFACTS
    // =======================
    private boolean collectArtifactInOrder(char targetArtifact) {
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return false;

        // Check for nearby obstacles
        ObstacleObservation obs = detectFrontObstacle();
        if (obs != null && obs.distanceIn < ROBOT_SAFETY_DISTANCE) {
            sidestepAroundRobot(obs);
            return false;
        }

        // Search for target color artifact
        List<LLResultTypes.DetectorResult> detections = res.getDetectorResults();
        if (detections != null) {
            for (LLResultTypes.DetectorResult d : detections) {
                String color = d.getClassName();
                if (matchesMotifArtifact(targetArtifact, color)) {
                    moveToArtifact(d.getTargetXDegrees(), d.getTargetYDegrees());
                    pickUpArtifact();
                    return true;
                }
            }
        }
        return false;
    }

    // =======================
    //  MOVEMENT HELPERS
    // =======================
    private void sidestepAroundRobot(ObstacleObservation obs) {
        Pose cur = follower.getPose();
        double sidestepX = (obs.txDeg < 0)
                ? cur.getX() + SIDESTEP_DISTANCE
                : cur.getX() - SIDESTEP_DISTANCE;

        Pose newPose = new Pose(sidestepX, cur.getY() + 5, cur.getHeading());
        replanPath(cur, newPose);
        waitForFollower();
    }

    private void moveToArtifact(double xOffset, double yOffset) {
        Pose cur = follower.getPose();
        Pose target = new Pose(cur.getX() + xOffset, cur.getY() + yOffset, cur.getHeading());
        replanPath(cur, target);
        waitForFollower();
    }

    private void goToShootingZone() {
        Pose target = isBlueAlliance ? blueShootingPose : redShootingPose;
        replanPath(follower.getPose(), target);
        waitForFollower();
    }

    private void goToParkingZone() {
        Pose target = isBlueAlliance ? blueParkPose : redParkPose;
        replanPath(follower.getPose(), target);
        waitForFollower();
    }

    // =======================
    //  PATH CONTROL
    // =======================
    private void replanPath(Pose start, Pose target) {
        follower.breakFollowing();
        follower.followPath(
                follower.pathBuilder().addPath(new BezierLine(start, target)).build()
        );
    }

    private void waitForFollower() {
        while (opModeIsActive() && follower.isBusy() && !isTimeUp()) {
            follower.update();
            detectCrashOrStuck();
            idle();
        }
    }

    // =======================
    //  ACTIONS
    // =======================
    private void pickUpArtifact() {
        telemetry.addLine("🤖 Picking up artifact...");
        telemetry.update();
        // TODO: add intake motor/servo control
        sleep(400);
    }

    private void shootArtifacts() {
        telemetry.addLine("🎯 Shooting artifacts...");
        telemetry.update();
        // TODO: add shooter control
        sleep(600);
    }

    // =======================
    //  OBSTACLE DETECTION
    // =======================
    private ObstacleObservation detectFrontObstacle() {
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
        if (fiducials == null) return null;

        ObstacleObservation closest = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (isObeliskTag(f.getFiducialId())) continue;

            double tx = f.getTargetXDegrees();
            double distance = Math.max(12, Math.abs(100 - tx)); // crude fallback
            ObstacleObservation obs = new ObstacleObservation(tx, distance, System.currentTimeMillis());

            if (closest == null || obs.distanceIn < closest.distanceIn) {
                closest = obs;
            }
        }
        return closest;
    }

    // =======================
    //  SAFETY & UTILITIES
    // =======================
    private boolean detectCrashOrStuck() {
        Pose current = follower.getPose();
        double distMoved = Math.hypot(
                current.getX() - lastPose.getX(),
                current.getY() - lastPose.getY()
        );
        long now = System.currentTimeMillis();

        if (distMoved < 1.0) {
            if (now - lastPoseUpdateTime > STUCK_TIMEOUT) {
                telemetry.addLine("⚠️ Robot appears stuck! Attempting recovery...");
                telemetry.update();

                replanPath(current, new Pose(current.getX() + 6, current.getY(), current.getHeading()));
                waitForFollower();

                lastPoseUpdateTime = now;
                return true;
            }
        } else {
            lastPose = current;
            lastPoseUpdateTime = now;
        }
        return false;
    }

    private boolean matchesMotifArtifact(char art, String color) {
        return (art == 'A' && color.equalsIgnoreCase("green")) ||
                (art == 'B' && color.equalsIgnoreCase("purple")) ||
                (art == 'C' && (color.equalsIgnoreCase("green") || color.equalsIgnoreCase("purple")));
    }

    private boolean isObeliskTag(int id) {
        return (id == 21 || id == 22 || id == 23);
    }

    private char[] motifFromTagId(int id) {
        switch (id) {
            case 21: return new char[]{'G', 'P', 'P'};
            case 22: return new char[]{'P', 'G', 'P'};
            case 23: return new char[]{'P', 'P', 'G'};
            default: return new char[]{'G', 'P', 'P'};
        }
    }

    private boolean isTimeUp() {
        return (System.currentTimeMillis() - startTime) > AUTON_TIMEOUT;
    }

    // =======================
    //  INNER CLASSES
    // =======================
    private static class ObstacleObservation {
        double txDeg;
        double distanceIn;
        long ts;

        ObstacleObservation(double tx, double dist, long t) {
            this.txDeg = tx;
            this.distanceIn = dist;
            this.ts = t;
        }
    }
}
//shooting code package org.firstinspires.ftc.teamcode.DECODE;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp
//public class BasicBotTeleop2 extends OpMode {
//
//    private DcMotor leftBack, leftFront, rightBack, rightFront;
//    private DcMotor shooter;
//    private CRServo geckoLeft, geckoRight;
//    private CRServo intake;
//
//    @Override
//    public void init() {
//        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
//        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
//        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
//        rightBack = hardwareMap.get(DcMotor.class,"rightBack");
//
//        shooter = hardwareMap.get(DcMotor.class,"shooter");
//
//        geckoLeft = hardwareMap.get(CRServo.class,"geckoLeft");
//        geckoRight = hardwareMap.get(CRServo.class,"geckoRight");
//        intake = hardwareMap.get(CRServo.class,"intake");
//
//        // Standard motor direction setup
//        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        // Stop all motors/servos initially
//        geckoLeft.setPower(0);
//        geckoRight.setPower(0);
//        intake.setPower(0);
//        shooter.setPower(0);
//    }
//
//    @Override
//    public void loop() {
//
//        // Drive controls
//        double x = gamepad2.right_stick_x * 0.3;
//        double y = gamepad2.left_stick_y * 1.1;
//        double rx = -gamepad2.left_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower  = (y + x + rx) / denominator;
//        double backLeftPower   = (y + x - rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower  = (y - x + rx) / denominator;
//
//        leftFront.setPower(frontLeftPower);
//        leftBack.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightBack.setPower(backRightPower);
//
//
//        double shooterPower = gamepad1.left_trigger*0.67;
//        shooter.setPower(shooterPower);
//
//        if(gamepad1.left_bumper){
//            shooter.setPower(-0.67);
//        }
//
//        // Intake + Gecko controls
//        if (gamepad1.b) {
//            intake.setPower(-1);
//            geckoLeft.setPower(1);
//            geckoRight.setPower(-1);
//        } else if (gamepad1.right_trigger>0.1) {
//            geckoRight.setPower(1);
//            geckoLeft.setPower(-1);
//            intake.setPower(1);
//        } else if (gamepad1.a) {
//            intake.setPower(0);
//            geckoLeft.setPower(-1);
//            geckoRight.setPower(1);
//        } else {
//            intake.setPower(0);
//            geckoLeft.setPower(0);
//            geckoRight.setPower(0);
//        }
//    }
//}