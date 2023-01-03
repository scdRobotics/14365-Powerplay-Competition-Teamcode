package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Autonomous(group = "advanced")
public class TestTrajBreak extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: -20, y: -35, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-20, -35, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Example spline path from SplineTest.java
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 60), 0)
                .build();

        // We follow the trajectory asynchronously so we can run our own logic
        drive.followTrajectoryAsync(traj);

        // Start the timer so we know when to cancel the following
        ElapsedTime stopTimer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            // 3 seconds into the opmode, we cancel the following
            if (stopTimer.seconds() >= 2) {
                // Cancel following
                drive.breakFollowing();

                // Stop the motors
                drive.setDrivePower(new Pose2d());
                drive.update();
                break;
            }

            // Update drive
            drive.update();
        }
    }

}
