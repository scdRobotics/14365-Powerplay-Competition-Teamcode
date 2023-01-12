package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BR_RL_AUTO", group="Autonomous")
public class _RIGHT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        delivery.initEncoders();

        vision.activateAprilTagYellowPipelineCamera1();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(-36, 63.5, Math.toRadians(270));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();

                    telemetry.addData("Approach Pole Complete! ", "");
                    telemetry.update();



                })

                .lineTo(new Vector2d(-36, 4))

                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))

                .build();

        delivery.closeGripper();

        waitForStart();

        int park = vision.readAprilTagCamera1() + 1;
        vision.activateYellowPipelineCamera2();

        boolean robotDetected = false;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.drive.followTrajectorySequenceAsync(approachPole);

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy() && !robotDetected){ //Should leave loop when async function is done or robot is detected

            /*if((sensors.getFrontLeftDist()<12 && sensors.getFrontLeftDist()>5) || (sensors.getFrontRightDist()<12 && sensors.getFrontRightDist()>5)){ //Meaning a robot is approaching the same direction
                robot.drive.breakFollowing();
                robot.drive.setDrivePower(new Pose2d());
                robotDetected=true;
                break;
            }*/

            // Update drive localization
            robot.drive.update();

        }

        if(robotDetected){
            //Run alt version of program (go for middle 5 point since our partner doesn't-- requires accurate scouting)

            TrajectorySequence altTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                    .lineToLinearHeading(new Pose2d(-36, 39.5, Math.toRadians(270)))

                    .build();

            robot.drive.followTrajectorySequence(altTraj);



        }
        else{

            //Vector2d highPole = new Vector2d(24, 0); //X and Y of high pole we stack on. Compare with dTheta and dist sensor to find absolute pos from relative localizer

            //double dTheta = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(approachPole.end())
                    //.turn(dTheta)
                    .turn(Math.toRadians(-1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist();
            if(distToPole>20){
                distToPole=6;
            }
            TrajectorySequence dropPolePickupNewCone = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, 13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(-135))

                    .lineToConstantHeading(new Vector2d(-68, 13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        //robot.pause(1);

                        delivery.closeGripper();

                        //robot.pause(1);



                    })

                    .lineTo(new Vector2d(-67, 13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slideHigh();
                    })



                    .lineTo(new Vector2d(-36, 13))

                    .turn(Math.toRadians(135))


                    .build();



            robot.drive.followTrajectorySequence(dropPolePickupNewCone);

            //double dTheta2 = vision.findClosePoleDTheta();

            TrajectorySequence turnToPole2 = robot.drive.trajectorySequenceBuilder(approachPole.end())
                    //.turn(dTheta2)
                    .turn(Math.toRadians(-1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole2);

            double distToPole2 = sensors.getFrontDist();
            if(distToPole2>20){
                distToPole2=6;
            }

            TrajectorySequence dropLastCone = robot.drive.trajectorySequenceBuilder(dropPolePickupNewCone.end())
                    .forward(distToPole2)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, 13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slidePickupStackSecond();

                    })



                    .turn(Math.toRadians(-135))

                    .build();



            robot.drive.followTrajectorySequence(dropLastCone);




            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-36, 13))

                        .build();

                robot.drive.followTrajectorySequence(park2);


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-60, 13))

                        .build();

                robot.drive.followTrajectorySequence(park3);


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-12, 13))

                        .build();

                robot.drive.followTrajectorySequence(park1);


            }

        }






        robot.pause(10);


    }

}
