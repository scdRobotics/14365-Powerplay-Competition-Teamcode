package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BLUE_RIGHT_AUTO", group="Autonomous")
public class _BLUE_RIGHT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        delivery.initEncoders();

        //vision.activateAprilTagYellowPipelineCamera1();

        vision.activateYellowPipelineCamera2();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(-36, 63.5, Math.toRadians(270));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();



                })

                .lineTo(new Vector2d(-36, 4))



                .build();


        TrajectorySequence alignPole = robot.drive.trajectorySequenceBuilder(approachPole.end())

                .lineToLinearHeading(new Pose2d(-36, 15, Math.toRadians(315)))

                .build();

        delivery.closeGripper();

        waitForStart();

        PoseTransfer.alliance = "BLUE";

        int park = vision.readAprilTagCamera1() + 1;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        vision.runAprilTag(false);

        boolean robotDetected = false;

        robot.drive.followTrajectorySequenceAsync(approachPole);

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy() && !robotDetected){ //Should leave loop when async function is done or robot is detected

            if((sensors.getFrontRightDist()<10 && sensors.getFrontRightDist()>5)){ //Meaning a robot is approaching the same direction
                robot.drive.breakFollowing();
                robot.drive.setDrivePower(new Pose2d());
                robotDetected=true;
                break;
            }

            // Update drive localization
            robot.drive.update();

        }

        if(robotDetected){
            //Run alt version of program (go for middle 4 point)

            TrajectorySequence altTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                    .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(270)))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slideMed();

                        telemetry.addData("Approach Pole Complete! ", "");
                        telemetry.update();



                    })

                    .turn(Math.toRadians(45))

                    .build();

            robot.drive.followTrajectorySequence(altTraj);


            //double dTheta = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(altTraj.end())
                    //.turn(dTheta)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist() - 0.5;
            if(distToPole>12){
                distToPole=6;
            }

            TrajectorySequence dropPoleMid = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, 37))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(-135))

                    .build();

            robot.drive.followTrajectorySequence(dropPoleMid);

            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-36, 37))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.park=2;


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-12, 37))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.park=3;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-60, 37))

                        .build();

                robot.drive.followTrajectorySequence(park1);

                PoseTransfer.park=1;


            }

            PoseTransfer.alt=true;


        }




















        else{

            //Vector2d highPole = new Vector2d(24, 0); //X and Y of high pole we stack on. Compare with dTheta and dist sensor to find absolute pos from relative localizer

            robot.drive.followTrajectorySequence(alignPole);

            //double dTheta = vision.findClosePoleDTheta();

            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(alignPole.end())
                    //.turn(dTheta)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist() - 1;
            if(distToPole>12){
                distToPole=7.5;
            }
            TrajectorySequence dropPolePickupNewCone = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, 15))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(-135))

                    .lineToConstantHeading(new Vector2d(-65, 15))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        //robot.pause(1);

                        delivery.closeGripper();

                        //robot.pause(1);



                    })

                    .lineTo(new Vector2d(-64.5, 15))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slideHigh();
                    })



                    .lineTo(new Vector2d(-36, 15))

                    .turn(Math.toRadians(135))


                    .build();



            robot.drive.followTrajectorySequence(dropPolePickupNewCone);

            //double dTheta2 = vision.findClosePoleDTheta();

            TrajectorySequence turnToPole2 = robot.drive.trajectorySequenceBuilder(dropPolePickupNewCone.end())
                    //.turn(dTheta2)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole2);

            double distToPole2 = sensors.getFrontDist();
            if(distToPole2>12){
                distToPole2=8.25;
            }

            TrajectorySequence dropLastCone = robot.drive.trajectorySequenceBuilder(turnToPole2.end())
                    .forward(distToPole2)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, 15))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slidePickupStackSecond();

                    })



                    .turn(Math.toRadians(-135))

                    .build();



            robot.drive.followTrajectorySequence(dropLastCone);




            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-36, 15))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.park=2;


            }

            else if(park==1){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-12, 15))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.park=3;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-60, 15))

                        .build();

                robot.drive.followTrajectorySequence(park1);

                PoseTransfer.park=1;


            }

            PoseTransfer.alt=false;

        }



        PoseTransfer.currentPose = robot.drive.getPoseEstimate();

        robot.pause(30);


    }

}
