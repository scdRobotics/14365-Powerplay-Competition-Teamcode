package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous(name="_RIGHT_AUTO", group="Autonomous")
public class _RIGHT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{

        initAuto();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(-START_X, START_Y, Math.toRadians(START_ANG));

        boolean robotDetected = false;

        robot.drive.setPoseEstimate(startPose);

        Pose2d DROP_POSE_ESTIMATE = new Pose2d(-32, 9, Math.toRadians(270+45));



        TrajectorySequence I_APPROACH = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(300, SLIDE_POWER);
                })

                .splineTo(new Vector2d(-I_APPROACH_X, I_APPROACH_Y - 6), Math.toRadians(270))

                .splineToSplineHeading(new Pose2d(-I_APPROACH_X, I_APPROACH_Y, Math.toRadians(315)), Math.toRadians(270))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .build();

        //Live build for drop off!

        TrajectorySequence II_APPROACH = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE) //Need to have estimate for startPose

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .splineToLinearHeading(new Pose2d(-I_BACK_POLE_X, I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG - 180)), Math.toRadians(180)) //Need to make sure this doesn't cause odo wheels to go on ground junction
                //.lineToLinearHeading(new Pose2d(-I_BACK_POLE_X, I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG - 180)))

                .splineToConstantHeading(new Vector2d(-I_PKUP_X, I_PKUP_Y), Math.toRadians(180))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                })

                .waitSeconds(STACK_WAIT_UP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT+700, SLIDE_POWER);
                })

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(-I_PKUP_BKUP_X, I_PKUP_BKUP_Y))

                .splineToConstantHeading(new Vector2d(-(II_APPROACH_X + 6), II_APPROACH_Y), Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(-II_APPROACH_X, II_APPROACH_Y, Math.toRadians(315)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .build();

        //Live build for drop off!


        TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(-II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 180))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(-PARK_II_X, PARK_Y))

                .build();


        TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(-II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 180))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(-LEFT_PARK_I_X, PARK_Y))

                .build();

        TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(-II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 180))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(-LEFT_PARK_III_X, PARK_Y))

                .build();





        robot.delivery.closeGripper();

        waitForStart();

        int park = robot.vision.readAprilTagCamera2() + 1;

        robot.vision.runAprilTag(false); //Hopefully this will decrease pipeline overhead time
        robot.vision.pauseCamera();

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.drive.followTrajectorySequence(I_APPROACH);

        robot.vision.resumeCamera();

        robot.pause(1.5);

        robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED);

        int loopCount = 0;

        double dTheta = robot.vision.findClosePoleDTheta();

        double dist = robot.vision.findClosePoleDist();

        while(loopCount<10 && (dTheta == -1 || dist == -1)){
            dTheta = robot.vision.findClosePoleDTheta();
            dist = robot.vision.findClosePoleDist();
            loopCount++;
            robot.pause(.175);
        }

        robot.vision.pauseCamera();


        TrajectorySequence I_DROP = robot.drive.trajectorySequenceBuilder(I_APPROACH.end())

                //.turn(dTheta * Math.abs(Math.cos(dTheta)))

                ////.turn(dTheta - Math.toRadians(5))
                .turn(dTheta)

                .forward(dist - 6.25)

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .forward(-6)

                .build();

        robot.drive.followTrajectorySequence(I_DROP);

        telemetry.addData("X Pos: ", robot.drive.getPoseEstimate().getX());
        telemetry.addData("Y Pos: ", robot.drive.getPoseEstimate().getY());
        telemetry.addData("Heading: ", robot.drive.getPoseEstimate().getHeading());
        telemetry.update();

        robot.drive.followTrajectorySequence(II_APPROACH);

        robot.vision.resumeCamera();

        robot.pause(1.5);

        robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED);

        loopCount = 0;

        dTheta = robot.vision.findClosePoleDTheta();

        dist = robot.vision.findClosePoleDist();

        while(loopCount<10 && (dTheta == -1 || dist == -1)){
            dTheta = robot.vision.findClosePoleDTheta();
            dist = robot.vision.findClosePoleDist();
            loopCount++;
            robot.pause(.175);
        }

        robot.vision.pauseCamera();


        TrajectorySequence II_DROP = robot.drive.trajectorySequenceBuilder(II_APPROACH.end())

                //.turn(dTheta - Math.toRadians(5))
                .turn(dTheta)

                .forward(dist - 6.25)

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })


                .forward(-6)

                .build();

        robot.drive.followTrajectorySequence(II_DROP);


        if(park==2){

            robot.drive.followTrajectorySequence(parkTwo);
        }
        else if(park == 3){

            robot.drive.followTrajectorySequence(parkThree);
        }
        else{

            robot.drive.followTrajectorySequence(parkOne);
        }

        PoseTransfer.currentPose = robot.drive.getPoseEstimate();
        PoseTransfer.slidePos = robot.delivery.getSlidePos();

        //Find actual closest coord grid values in case park goes wrong, and also prevents a code block for each block case
        //TODO: NEEDS TESTING TO ENSURE IT ACTUALLY WORKS PROPERLY


        PoseTransfer.idealGridAngle = robot.drive.getPoseEstimate().getHeading();



        robot.pause(30);


    }


}
