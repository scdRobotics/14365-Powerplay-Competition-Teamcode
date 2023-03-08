package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="MotorTest", group="TestAutonomous")
public class MotorTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();

        initAuto();

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANG));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence I_APPROACH = robot.drive.trajectorySequenceBuilder(startPose)

                .lineTo(new Vector2d(I_APPROACH_X, I_APPROACH_Y - 6))

                .build();

        waitForStart();

        robot.drive.followTrajectorySequence(I_APPROACH);






    }

}
