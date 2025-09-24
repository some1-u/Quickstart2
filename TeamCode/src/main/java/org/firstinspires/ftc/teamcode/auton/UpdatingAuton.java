package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "DynamicObstacleAvoiding", group = "Autonomous")
public class UpdatingAuton extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);
        Limelight3A limelight;
        int[] motif;
        waitForStart();
        //initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);  // Make sure pipeline 0 is the AprilTag pipeline
        limelight.start();
        // Start with an initial path
        while(true){
            LLResult result = limelight.getLatestResult();
            boolean obeliskDetected = false;
            double tx = 0; // horizontal offset
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (result != null && result.isValid()) {
                boolean aprilTagDetected = true;
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() != 21 && fr.getFiducialId() != 22 && fr.getFiducialId() != 23){
                        Pose currentPose = follower.getPose();
                        Pose motifDetour = new Pose(
                                currentPose.getX(),
                                currentPose.getY() + 10,
                                currentPose.getHeading()
                        );
                        }
                    else{
                        int[][] motifs = {{0,1,1},{1,0,1},{1,1,0}};
                         motif = motifs[fr.getFiducialId() % 3];                    }
                    }
            }
        }
        // Main loop
        int artifactColor=motif[0];
        while (opModeIsActive()) {
            follower.update();

            // ---- Detection Logic ----
            if (aprilTagDetected()) {
                telemetry.addLine("AprilTag detected!");
                telemetry.update();
                // add behavior here if needed
            }

            if (artifactDetected()) {
                telemetry.addLine("Artifact detected!");
                telemetry.update();
                // use artifactDetectionDistance() for dynamic offset
                Pose currentPose = follower.getPose();
                int[] offset = artifactDetectionDistance();
                Pose detour = new Pose(
                        currentPose.getX() + offset[0],
                        currentPose.getY() + offset[1],
                        currentPose.getHeading()
                );
                replanPath(currentPose, detour);
            }

            if (robotDetectedInPath()) {
                telemetry.addLine("⚠️ Obstacle detected! Replanning...");
                telemetry.update();

                Pose currentPose = follower.getPose();
                // simple forward sidestep
                Pose detour = new Pose(
                        currentPose.getX() + 10,
                        currentPose.getY() + 10,
                        currentPose.getHeading()
                );
                replanPath(currentPose, detour);
            }

            idle();
        }
    }

    // ---------------- Utility Methods ----------------
    private void replanPath(Pose currentPose, Pose detour) {
        follower.breakFollowing();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, detour))
                        .build()
        );
    }

    // ----- Detection Functions -----
    private boolean aprilTagDetected() {
        return false; // TODO: integrate Limelight AprilTag logic
    }

    private boolean artifactDetected(int color) {
        return false; // TODO: vision/artifact detection
    }

    private int[] artifactDetectionDistance() {
        return new int[]{0, 0}; // placeholder
    }

    private boolean robotDetectedInPath() {
        return gamepad1.a; // placeholder for sensor detection
    }

    private int[] motifDetection() {
        int[][] motifs = {{0,1,1},{1,0,1},{1,1,0}};
        int sensorNum = 21 % 3;
        return motifs[sensorNum];
    }
}
