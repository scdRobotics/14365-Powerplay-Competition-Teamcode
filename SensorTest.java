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

@Autonomous(name="SensorTest", group="TestAutonomous")
public class SensorTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();

        initAuto();

        waitForStart();

        /*while(!isStopRequested()){
            telemetry.addData("Front Right sensor readout: ", robot.sensors..getFrontRightDist());
            telemetry.addData("Front Left sensor readout: ", robot.sensors..getFrontLeftDist());
            telemetry.addData("Front sensor readout: ", robot.sensors..getFrontDist());
            telemetry.update();
        }*/

        /*

        ---BOTH---
        BLUE ALLIANCE: SKY_BLUE (Brightest blue)
        RED ALLIANCE: RED (Brightest red)

        ---AUTO---
        IN SYNC WITH TRAJ: CONSTANT
        DESYNCED WITH TRAJ: HEARTBEAT

        ---TELEOP---
        (WHEN SLIDE IS EXTENDED UP)
        READY TO STACK: GREEN
        NOT READY TO STACK: YELLOW

        (DEPENDENT ON MODE)
        AUTO: PURPLE
        MANUAL: SKY_BLUE/RED (ALLIANCE COLORS)





         */


        while(!isStopRequested()){

            robot.sensors.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            telemetry.addData("YELLOW", "");
            telemetry.update();

            robot.pause(3);

            robot.sensors.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            telemetry.addData("VIOLET", "");
            telemetry.update();

            robot.pause(3);

        }








    }

}
