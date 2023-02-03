package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="VisionTest", group="Autonomous")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {




        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;

        vision.activateYellowPipelineCamera1();
        vision.activateAprilTagYellowPipelineCamera2();



        waitForStart();

        //vision.runAprilTag(false);

        telemetry.addData("Calculate dTheta: ", vision.findClosePoleDTheta());
        telemetry.update();



        while(opModeIsActive() && !isStopRequested()){

            telemetry.addData("April Tag ID Detected: ", vision.readAprilTagCamera2());

            /*for(int i = 0; i<vision.same.size(); i++){
                telemetry.addData("Same  Value " + i + " X: ", vision.same.get(i).getX());
                telemetry.addData("Same  Value " + i + " Y: ", vision.same.get(i).getY());
                telemetry.addData("Same  Value " + i + " Width: ", vision.same.get(i).getWidth());
                telemetry.addData("Same  Value " + i + " Height: ", vision.same.get(i).getHeight());
            }*/


        }










        //robot.drive.turn(Math.toRadians(-38));

        /*telemetry.addData("Approach Pole Complete! ", "");
        telemetry.update();*/

        /*while( (sensors.getFrontDist()>10 || sensors.getFrontDist()<6) && failsafeCount<10 && !isStopRequested()){
            robot.drive.turn(Math.toRadians(-9));
            failsafeCount++;
            telemetry.addData("In sensors loop with failsafe count of:  ", failsafeCount);
            telemetry.addData("Distance sensor reads:  ", sensors.getFrontDist());
            telemetry.update();
        }*/











        //robot.drive.followTrajectorySequence(driveIntoPole);



        robot.pause(5);






    }

}
