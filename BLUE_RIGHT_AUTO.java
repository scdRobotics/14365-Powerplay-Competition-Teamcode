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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BLUE_RIGHT_AUTO", group="Autonomous")
public class BLUE_RIGHT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {




        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        double dist = 0.1;



        vision.activateAprilTagYellowPipelineCamera1();

        robot.drive.setPoseEstimate(new Pose2d(0,0,0));

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))


                .forward(55)
                //.lineToLinearHeading(new Pose2d(57, 0, Math.toRadians(45)))
                .turn(Math.toRadians(-55)) //315?

                .forward(10)

                .build();


        TrajectorySequence turnCheckPole = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))

                .turn(Math.toRadians(-3)) //315?

                .build();

        delivery.closeGripper();

        waitForStart();

        int park = vision.readAprilTagCamera1() + 1;

        vision.activateYellowPipelineCamera2();


        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        delivery.slideHigh();

        robot.drive.followTrajectorySequence(approachPole);

        int failsafeCount = 0;
        while(sensors.getFrontDist()<10 && sensors.getFrontDist()>1.5 && failsafeCount<=7){
            robot.drive.followTrajectorySequence(turnCheckPole);
            failsafeCount++;
        }

        dist = sensors.getFrontDist()-3;

        TrajectorySequence driveIntoPole = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))

                .turn(Math.toRadians(-3))

                .forward(Math.toRadians(dist)) //315?

                .build();

        robot.drive.followTrajectorySequence(driveIntoPole);




        delivery.openGripper();

        robot.pause(5);






    }

}
