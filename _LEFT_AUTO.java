package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous(name="_LEFT_AUTO", group="Autonomous")
public class _LEFT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{

        initAuto();
        robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANG));

        boolean robotDetected = false;

        robot.drive.setPoseEstimate(startPose);

        Pose2d DROP_POSE_ESTIMATE = new Pose2d(29, 6, Math.toRadians(270-45));



        TrajectorySequence I_APPROACH = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .splineTo(new Vector2d(I_APPROACH_X, I_APPROACH_Y - 6), Math.toRadians(270))

                .splineToSplineHeading(new Pose2d(I_APPROACH_X, I_APPROACH_Y, Math.toRadians(225)), Math.toRadians(270))

                .build();

        //Live build for drop off!

        TrajectorySequence II_APPROACH = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE) //Need to have estimate for startPose

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .splineToLinearHeading(new Pose2d(I_BACK_POLE_X, I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG)), Math.toRadians(0)) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .splineToConstantHeading(new Vector2d(I_PKUP_X, I_PKUP_Y), Math.toRadians(0))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                })

                .waitSeconds(STACK_WAIT_UP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(I_PKUP_BKUP_X, I_PKUP_BKUP_Y))

                .splineToConstantHeading(new Vector2d(II_APPROACH_X + 8, II_APPROACH_Y), Math.toRadians(180))

                .splineToSplineHeading(new Pose2d(II_APPROACH_X, II_APPROACH_Y, Math.toRadians(225)), Math.toRadians(180))

                .build();

        //Live build for drop off!

        /*TrajectorySequence III_APPROACH = robot.drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(II_PKUP_X, II_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(II_PKUP_BKUP_X, II_PKUP_BKUP_Y))

                .lineTo(new Vector2d(III_APPROACH_X, III_APPROACH_Y))

                .turn(Math.toRadians(-III_APPROACH_TURN))

                .build();*/



        TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(PARK_II_X, PARK_Y))

                .build();


        TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(LEFT_PARK_III_X, PARK_Y))

                .build();

        TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(DROP_POSE_ESTIMATE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                })

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(LEFT_PARK_I_X, PARK_Y))

                .build();





        robot.delivery.closeGripper();

        waitForStart();

        int park = robot.vision.readAprilTagCamera2() + 1;

        robot.vision.runAprilTag(false); //Hopefully this will decrease pipeline overhead time

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.vision.runAprilTag(false);

        robot.drive.followTrajectorySequence(I_APPROACH);

        robot.pause(1.5);

        robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED);

        double dTheta = 0;
        double dist = 0;

        ArrayList<Double>  dThetas = new ArrayList<>();
        ArrayList<Double>  dists = new ArrayList<>();

        int count = 0;

        while(count<20){

            if(dThetas.size() < 10){
                dTheta = robot.vision.findClosePoleDTheta();
                if(dTheta!=-1 && dTheta>Math.toRadians(-30) && dTheta<Math.toRadians(30)){
                    dThetas.add(dTheta);
                }
            }

            if(dists.size() < 10){
                dist = robot.vision.findClosePoleDist();
                if(dist!=-1 && dist<23 && dist>8){
                    dists.add(dist);
                }
            }

            if(dThetas.size()==10 && dists.size()==10){
                break;
            }

            count++;

            telemetry.addData("Loop Count: ", count);


            robot.pause(0.075);
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


        TrajectorySequence I_DROP = robot.drive.trajectorySequenceBuilder(I_APPROACH.end())

                //.turn(dTheta * Math.abs(Math.cos(dTheta)))

                .turn(dTheta * Math.abs(Math.cos(dTheta)))

                .forward(dist - 7 + 1.75)

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

        telemetry.addData("X Pos: ", robot.drive.getPoseEstimate().getX());
        telemetry.addData("Y Pos: ", robot.drive.getPoseEstimate().getY());
        telemetry.addData("Heading: ", robot.drive.getPoseEstimate().getHeading());
        telemetry.update();

        robot.drive.followTrajectorySequence(II_APPROACH);

        robot.pause(1.5);

        robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED);

        dTheta = 0;
        dist = 0;

        dThetas.clear();
        dists.clear();

        count = 0;

        while(count<20){

            if(dThetas.size() < 10){
                dTheta = robot.vision.findClosePoleDTheta();
                if(dTheta!=-1 && dTheta>Math.toRadians(-30) && dTheta<Math.toRadians(30)){
                    dThetas.add(dTheta);
                }
            }

            if(dists.size() < 10){
                dist = robot.vision.findClosePoleDist();
                if(dist!=-1 && dist<23 && dist>8){
                    dists.add(dist);
                }
            }

            if(dThetas.size()==10 && dists.size()==10){
                break;
            }

            count++;

            telemetry.addData("Loop Count: ", count);


            robot.pause(0.075);
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

        if (dThetas.size() % 2 == 0)
            dTheta = (dThetas.get(dThetas.size()/2) + (dThetas.get(dThetas.size()/2-1)))/2;
        else
            dTheta = (dThetas.get(dThetas.size()/2));


        if (dists.size() % 2 == 0)
            dist = (dists.get(dists.size()/2) + (dists.get(dists.size()/2-1)))/2;
        else
            dist = (dists.get(dists.size()/2));


        TrajectorySequence II_DROP = robot.drive.trajectorySequenceBuilder(II_APPROACH.end())

                //.turn(dTheta - Math.toRadians(3.5))
                //.turn(dTheta * Math.abs(Math.cos(dTheta)))

                .turn(dTheta * Math.abs(Math.cos(dTheta)))

                .forward(dist - 7 + 1.75)

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

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
