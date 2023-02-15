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

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="VisionTest", group="Autonomous")
public class VisionTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {



        initAuto();


        robot.vision.runAprilTag(false);


        robot.drive.setPoseEstimate(new Pose2d(0,0,0));

        robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);

        waitForStart();


        double dTheta = 0;
        double dist = 0;

        ArrayList<Double> dThetas = new ArrayList<>();
        ArrayList<Double>  dists = new ArrayList<>();

        for(int i = 0; i<10; i++){
            dTheta = robot.vision.findClosePoleDTheta();
            dist = robot.vision.findClosePoleDist();
            if(dTheta!=-1){
                dThetas.add(dTheta);
            }
            if(dist!=-1){
                dists.add(dist);
            }
            robot.pause(.110);
        }

        for(Double d: dThetas){
            telemetry.addData("dTheta val: ", d);
        }

        for(Double d: dists){
            telemetry.addData("dist val: ", d);
        }

        telemetry.update();

        Collections.sort(dThetas);
        Collections.sort(dists);

        //We only take median rn, is there a better way? Probably.

        if (dThetas.size() % 2 == 0)
            dTheta = (dThetas.get(dThetas.size()/2) + (dThetas.get(dThetas.size()/2-1)))/2;
        else
            dTheta = (dThetas.get(dThetas.size()/2));


        if (dists.size() % 2 == 0)
            dist = (dists.get(dists.size()/2) + (dists.get(dists.size()/2-1)))/2;
        else
            dist = (dists.get(dists.size()/2));


        TrajectorySequence I_DROP = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))

                .turn(dTheta * Math.abs(Math.cos(dTheta))) //just using cosine to approximate better than a constant multiple

                .forward(dist - 7.2)

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .build();

        robot.drive.followTrajectorySequence(I_DROP);

        //vision.runAprilTag(false);
        telemetry.update();



        while(opModeIsActive() && !isStopRequested()){

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
