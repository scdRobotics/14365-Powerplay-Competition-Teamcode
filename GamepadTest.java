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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="GamepadTest", group="TestAutonomous")
public class GamepadTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();



        initAuto();



        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

                 gamepad1.rumble(0,0,750);

                 robot.pause(1);

                 gamepad1.rumble(0.1,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.2,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.3,0,750);
                 robot.pause(1);

                 gamepad1.rumble(0.4,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.5,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.6,0,750);
                 robot.pause(1);

                 gamepad1.rumble(0.7,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.8,0,750);
                 robot.pause(1);
                 gamepad1.rumble(0.9,0,750);
                 robot.pause(1);

                 gamepad1.rumble(1,0,750);
                 robot.pause(1);

                 gamepad1.rumbleBlips(5);
                 robot.pause(5);




            gamepad1.rumble(0,0,750);

            robot.pause(1);

            gamepad1.rumble(0.1,0.1,750);
            robot.pause(1);
            gamepad1.rumble(0.2,0.2,750);
            robot.pause(1);
            gamepad1.rumble(0.3,0.3,750);
            robot.pause(1);

            gamepad1.rumble(0.4,0.4,750);
            robot.pause(1);
            gamepad1.rumble(0.5,0.5,750);
            robot.pause(1);
            gamepad1.rumble(0.6,0.6,750);
            robot.pause(1);

            gamepad1.rumble(0.7,0.7,750);
            robot.pause(1);
            gamepad1.rumble(0.8,0.8,750);
            robot.pause(1);
            gamepad1.rumble(0.9,0.9,750);
            robot.pause(1);

            gamepad1.rumble(1,1,750);
            robot.pause(1);

            gamepad1.rumbleBlips(5);
            robot.pause(5);
        }




    }

}
