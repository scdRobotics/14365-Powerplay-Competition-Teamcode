package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="BLUE_RIGHT_AUTO", group="Autonomous")
public class BLUE_RIGHT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {




        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        delivery.initEncoders();

        vision.activateAprilTagYellowPipelineCamera1();

        robot.drive.setPoseEstimate(new Pose2d(0,0,0));

        //AtomicReference<Double> dist = new AtomicReference<>(5.0);



        TrajectorySequence poleApproach = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .forward(51)
                .strafeRight(10)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();

                    telemetry.addData("Approach Pole Complete! ", "");
                    telemetry.update();

                    int failsafeCount = 0;



                    while( (sensors.getFrontDist()>11 || sensors.getFrontDist()<6) && failsafeCount<10 && !isStopRequested()){
                        robot.drive.turn(Math.toRadians(-9));
                        failsafeCount++;
                        telemetry.addData("In sensors loop with failsafe count of:  ", failsafeCount);
                        telemetry.addData("Distance sensor reads:  ", sensors.getFrontDist());
                        telemetry.update();
                    }



                })


                .build();




        delivery.closeGripper();

        waitForStart();

        int park = vision.readAprilTagCamera1() + 1;
        vision.activateYellowPipelineCamera2();

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.drive.followTrajectorySequence(poleApproach);

        //dist.set(sensors.getFrontDist());
        double dist = sensors.getFrontDist() - 4;

        //double dist = (sensors.getFrontDist() * (Math.sin(Math.abs(robot.drive.getRawExternalHeading())))) - 0.25;

        telemetry.addData("Final distance readout to pole: ", dist);
        telemetry.addData("Heading: ", Math.abs(robot.drive.getRawExternalHeading()));
        telemetry.update();

        TrajectorySequence poleDrop = robot.drive.trajectorySequenceBuilder(poleApproach.end())

                .forward(dist)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    telemetry.addData("Drive into pole traj sequence done! ", "");
                    telemetry.update();

                    robot.pause(1);

                    delivery.openGripper();

                    telemetry.addData("Gripper opened! ", "");
                    telemetry.update();

                    robot.pause(1);


                })

                .lineTo(new Vector2d((51+dist)-10, 0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    delivery.slidePickupStack();
                })

                .turn(Math.toRadians(90))

                .lineTo(new Vector2d((51+dist)-10, 10))

                //.strafeLeft(20)

                .build();


        robot.drive.followTrajectorySequence(poleDrop);

        /*TrajectorySequence conePickup = robot.drive.trajectorySequenceBuilder(poleDrop.end())

                .build();*/


        robot.pause(10);










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
