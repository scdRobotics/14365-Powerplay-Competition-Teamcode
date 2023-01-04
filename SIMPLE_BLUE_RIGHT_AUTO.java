package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="SIMPLE_BLUE_RIGHT_AUTO", group="Autonomous")
public class SIMPLE_BLUE_RIGHT_AUTO extends LinearOpMode {

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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();

                    telemetry.addData("Approach Pole Complete! ", "");
                    telemetry.update();



                })
                .forward(52)
                .strafeRight(12)

                .forward(5)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    telemetry.addData("Drive into pole traj sequence done! ", "");
                    telemetry.update();

                    robot.pause(1);

                    delivery.openGripper();

                    telemetry.addData("Gripper opened! ", "");
                    telemetry.update();

                    robot.pause(1);


                })

                .lineTo(new Vector2d(48, -15))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    delivery.slidePickupStack();
                })

                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(50, 29.5))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.pause(1);

                    delivery.closeGripper();

                    robot.pause(1);

                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();

                })

                .lineTo(new Vector2d(50, -11.25))

                .turn(Math.toRadians(-90))

                .forward(6)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    telemetry.addData("Drive into pole traj sequence done! ", "");
                    telemetry.update();

                    robot.pause(1);

                    delivery.openGripper();

                    telemetry.addData("Gripper opened! ", "");
                    telemetry.update();

                    robot.pause(1);


                })



                .build();


        //PARK TRAJECTORY SEQUENCES NEED TESTING AND TUNING



        TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(poleApproach.end())

                .lineTo(new Vector2d(48, -15))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    delivery.slidePickupStackSecond();
                })

                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(50, 29.5))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.pause(1);

                    delivery.closeGripper();

                    robot.pause(1);

                })

                .build();


        TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(poleApproach.end())

                .lineTo(new Vector2d(48, -15))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    delivery.slidePickupStackSecond();
                })

                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(50, 0))

                .build();

        TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(poleApproach.end())

                .lineTo(new Vector2d(48, -15))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    delivery.slidePickupStackSecond();
                })

                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(50, -22.5))

                .build();


        delivery.closeGripper();

        waitForStart();

        int park = vision.readAprilTagCamera1() + 1;
        vision.activateYellowPipelineCamera2();

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.drive.followTrajectorySequence(poleApproach);

        if(park==1){
            robot.drive.followTrajectorySequence(parkOne);
        }
        else if(park==2){
            robot.drive.followTrajectorySequence(parkTwo);
        }
        else if(park==3){
            robot.drive.followTrajectorySequence(parkThree);
        }
        else{
            robot.drive.followTrajectorySequence(parkOne);
        }

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
