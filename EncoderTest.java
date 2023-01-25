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

@Autonomous(name="EncoderTest", group="Autonomous")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {




        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);



        waitForStart();

        //vision.runAprilTag(false);



        while(opModeIsActive() && !isStopRequested()){

        }



        robot.pause(5);






    }

}
