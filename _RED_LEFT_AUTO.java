package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RED_LEFT_AUTO", group="Autonomous")
public class _RED_LEFT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        delivery.initEncoders();

        vision.activateAprilTagYellowPipelineCamera1();

        vision.activateYellowPipelineCamera2();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(-36, -63.5, Math.toRadians(90));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();



                })

                .lineTo(new Vector2d(-36, -4))



                .build();


        TrajectorySequence alignPole = robot.drive.trajectorySequenceBuilder(approachPole.end())

                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))

                .build();

        delivery.closeGripper();

        waitForStart();

        PoseTransfer.alliance = "RED";

        int park = vision.readAprilTagCamera1() + 1;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        vision.runAprilTag(false);

        boolean robotDetected = false;

        robot.drive.followTrajectorySequenceAsync(approachPole);

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy() && !robotDetected){ //Should leave loop when async function is done or robot is detected

            if((sensors.getFrontLeftDist()<12 && sensors.getFrontLeftDist()>5) || (sensors.getFrontRightDist()<12 && sensors.getFrontRightDist()>5)){ //Meaning a robot is approaching the same direction
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

                    .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slideMed();

                        telemetry.addData("Approach Pole Complete! ", "");
                        telemetry.update();



                    })

                    .turn(-45)

                    .build();

            robot.drive.followTrajectorySequence(altTraj);


            //double dTheta = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(altTraj.end())
                    //.turn(dTheta)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist();
            if(distToPole>20){
                distToPole=6;
            }

            TrajectorySequence dropPoleMid = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, -37))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(135))

                    .build();

            robot.drive.followTrajectorySequence(dropPoleMid);

            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-36, -37))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.park=2;


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-12, -37))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.park=3;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(-60, -37))

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

            double distToPole = sensors.getFrontDist();
            if(distToPole>20){
                distToPole=6;
            }
            TrajectorySequence dropPolePickupNewCone = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(-30, -13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(135))

                    .lineToConstantHeading(new Vector2d(-68, -13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        //robot.pause(1);

                        delivery.closeGripper();

                        //robot.pause(1);



                    })

                    .lineTo(new Vector2d(-67.5, -13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slideHigh();
                    })



                    .lineTo(new Vector2d(-36, -13))

                    .turn(Math.toRadians(-135))


                    .build();



            robot.drive.followTrajectorySequence(dropPolePickupNewCone);

            //double dTheta2 = vision.findClosePoleDTheta();

            TrajectorySequence turnToPole2 = robot.drive.trajectorySequenceBuilder(approachPole.end())
                    //.turn(dTheta2)
                    .turn(Math.toRadians(1))
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

                    .lineToConstantHeading(new Vector2d(-30, -13))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slidePickupStackSecond();

                    })



                    .turn(Math.toRadians(135))

                    .build();



            robot.drive.followTrajectorySequence(dropLastCone);




            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-36, -13))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.park=2;


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-12, -13))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.park=3;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(-60, -13))

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
