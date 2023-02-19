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

@Autonomous(name="KalmanThetaTest", group="TestTeleop")
public class KalmanThetaTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);


        double x = 0; // your initial state
        double Q = 0.75; // your model covariance, about 0.75
        double R = 0.9; // your sensor covariance, this is another value that needs to be figured out and used as a constant, 0.9
        double p = 0.7; // your initial covariance guess
        double K = 1; // your initial Kalman gain guess

        double x_previous = x;
        double p_previous = p;
        double u = robot.sensors.getIMUReadout();
        double z = robot.drive.getPoseEstimate().getHeading();


        waitForStart();

        double drive_y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double drive_x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double drive_rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive_y) + Math.abs(x) + Math.abs(drive_rx), 1);

        double frontLeftPower = (drive_y + drive_x + drive_rx) / denominator;
        double backLeftPower = (drive_y - drive_x + drive_rx) / denominator;
        double frontRightPower = (drive_y - drive_x - drive_rx) / denominator;
        double backRightPower = (drive_y + drive_x - drive_rx) / denominator;

        while(!isStopRequested()){

            if(drive_y<0.075 || drive_x<0.0825 || drive_rx<0.075){ //Account for potential joystick drift
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y/3,
                                gamepad1.left_stick_x/3,
                                -gamepad1.right_stick_x/3
                        )
                );
            }

            u = robot.sensors.getIMUReadout();
            x = x_previous + u; //may not be necessary since IMu should save its prev readout and auto add? but we'll see
            p = p_previous + Q;
            K = p/(p + R);

            z = robot.drive.getPoseEstimate().getHeading(); //get "actual" heading value
            x = x + K * (z - x);
            p = (1 - K) * p;

            x_previous = x;
            p_previous = p;



        }








    }

}
