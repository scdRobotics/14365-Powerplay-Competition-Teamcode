package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RED_LEFT_AUTO", group="Autonomous")
public class _RED_LEFT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{

        PoseTransfer.isBlue=false;

        initAuto();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(START_X, -START_Y, Math.toRadians(START_ANG));

        boolean robotDetected = false;

        TrajectorySequence firstApproach = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .lineTo(new Vector2d(I_APPROACH_X, -I_APPROACH_Y))

                .build();

        TrajectorySequence idealTrajectory = robot.drive.trajectorySequenceBuilder(firstApproach.end())

                .turn(Math.toRadians(I_APPROACH_TURN))

                .lineTo(new Vector2d(I_DROP_X, -I_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //TODO: ADD REDUNDANCY CHECKS AGAINST THESE SENSORS. WILL PROBABLY WANT TO MOVE INTO A BOOLEAN FUNCTION SO WE CAN HAVE SEVERAL LINES AND KEEP EVERYTHING MUCH, MUCH CLEANER.
                    if( (robot.vision.findClosePoleDTheta() > Math.toRadians(WEBCAM_DEGREE_TOLERANCE)) || ( Math.abs(robot.sensors.getFrontDist() - I_EXPECTED_SENSOR_READOUT) > I_DISTANCE_SENSOR_TOLERANCE) || ( Math.abs(robot.vision.findClosePoleDist() - I_EXPECTED_WEBCAM_READOUT) > I_WEBCAM_DIST_TOLERANCE)){ //MAY add an additional check against distance sensors and webcam dist? Not sure if necessary yet, though
                        trajectorySkewFirst = true;
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(I_BACK_POLE_X, -I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG + 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(I_PKUP_X, -I_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(I_PKUP_BKUP_X, -I_PKUP_BKUP_Y))

                .lineTo(new Vector2d(II_APPROACH_X, -II_APPROACH_Y))

                .turn(Math.toRadians(-II_APPROACH_TURN))

                .lineTo(new Vector2d(II_DROP_X, -II_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //TODO: ADD REDUNDANCY CHECKS AGAINST THESE SENSORS. WILL PROBABLY WANT TO MOVE INTO A BOOLEAN FUNCTION SO WE CAN HAVE SEVERAL LINES AND KEEP EVERYTHING MUCH, MUCH CLEANER.
                    if( (robot.vision.findClosePoleDTheta() > Math.toRadians(WEBCAM_DEGREE_TOLERANCE)) || ( Math.abs(robot.sensors.getFrontDist() - II_III_EXPECTED_SENSOR_READOUT) > II_III_DISTANCE_SENSOR_TOLERANCE) || ( Math.abs(robot.vision.findClosePoleDist() - II_III_EXPECTED_WEBCAM_READOUT) > II_III_WEBCAM_DIST_TOLERANCE)){ //MAY add an additional check against distance sensors and webcam dist? Not sure if necessary yet, though
                        trajectorySkewSecond = true;
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, -II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG))) //Need to make sure this doesn't cause odo wheels to go on ground junction



                .lineTo(new Vector2d(II_PKUP_X, -II_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(II_PKUP_BKUP_X, -II_PKUP_BKUP_Y))

                .waitSeconds(STACK_WAIT_UP)


                .lineTo(new Vector2d(III_APPROACH_X, -III_APPROACH_Y))

                .turn(Math.toRadians(-III_APPROACH_TURN))


                .lineTo(new Vector2d(III_DROP_X, -III_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //TODO: ADD REDUNDANCY CHECKS AGAINST THESE SENSORS. WILL PROBABLY WANT TO MOVE INTO A BOOLEAN FUNCTION SO WE CAN HAVE SEVERAL LINES AND KEEP EVERYTHING MUCH, MUCH CLEANER.
                    if( (robot.vision.findClosePoleDTheta() > Math.toRadians(WEBCAM_DEGREE_TOLERANCE)) || ( Math.abs(robot.sensors.getFrontDist() - II_III_EXPECTED_SENSOR_READOUT) > II_III_DISTANCE_SENSOR_TOLERANCE) || ( Math.abs(robot.vision.findClosePoleDist() - II_III_EXPECTED_WEBCAM_READOUT) > II_III_WEBCAM_DIST_TOLERANCE)){ //MAY add an additional check against distance sensors and webcam dist? Not sure if necessary yet, though
                        trajectorySkewThird = true;
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(III_BACK_POLE_X, -III_BACK_POLE_Y, Math.toRadians(III_BACK_POLE_ANG))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .build();

        waitForStart();

        int park = robot.vision.readAprilTagCamera2() + 1;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.vision.runAprilTag(false);


        double frontSensorReadout = 0;
        double backSensorReadout = 0;

        robot.timer.startTime();

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

            if(robot.timer.time()>1.5){
                //frontSensorReadout = robot.sensors.getLeftFrontDist();
                //backSensorReadout = robot.sensors.getLeftBackDist();
            }


            if((frontSensorReadout<COLLISION_AVOIDANCE_UPPER_LIMIT && frontSensorReadout>COLLISION_AVOIDANCE_LOWER_LIMIT) && (backSensorReadout<COLLISION_AVOIDANCE_UPPER_LIMIT && backSensorReadout>COLLISION_AVOIDANCE_LOWER_LIMIT)){ //Meaning a robot is approaching the same direction
                robot.drive.breakFollowing();
                robot.drive.setDrivePower(new Pose2d());
                robotDetected=true;
                break;
            }

            // Update drive localization
            robot.drive.update();

        }


        if(robotDetected){
            //TODO: Run alt version of program (go for middle 4 point & park). Still needs programming.


        }




        else {


            robot.drive.followTrajectorySequenceAsync(idealTrajectory);
            while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

                if(trajectorySkewFirst || trajectorySkewSecond || trajectorySkewThird){ //Meaning we sense we are off of the trajectory by a significant margin. One for each pole drop
                    robot.drive.breakFollowing();
                    robot.drive.setDrivePower(new Pose2d());
                    robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED);
                    break;
                }

                // Update drive localization
                robot.drive.update();

            }

            //TODO: ADD "LIVE-BUILT" TRAJECTORIES HERE IN CASE OF SKEW

            if(park==2){
                TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(-PARK_II_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkTwo);
            }
            else if(park == 3){
                TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(-LEFT_PARK_III_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkThree);
            }
            else{
                TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(-LEFT_PARK_I_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkOne);
            }

        }

        PoseTransfer.currentPose = robot.drive.getPoseEstimate();
        PoseTransfer.slidePos = robot.delivery.getSlidePos();

        //Find actual closest coord grid values in case park goes wrong, and also prevents a code block for each block case
        //TODO: NEEDS TESTING TO ENSURE IT ACTUALLY WORKS PROPERLY
        int closestX = 0;
        double closestTempValX = 100;
        int closestY = 0;
        double closestTempValY = 100;
        int closestAngle = 0;
        double closestTempValAngle = 500;
        for(int i = 0; i< 6; i++){
            if(Math.abs(validRobotPosConversion[i]-robot.drive.getPoseEstimate().getX()) < closestTempValX){
                closestTempValX = robot.drive.getPoseEstimate().getX();
                closestX = i;
            }
        }

        for(int i = 0; i< 6; i++){
            if(Math.abs(validRobotPosConversion[i]-robot.drive.getPoseEstimate().getY()) < closestTempValY){
                closestTempValY = robot.drive.getPoseEstimate().getY();
                closestY = i;
            }
        }

        for(int i = 0; i<360; i+=90){
            if(Math.abs(robot.drive.getPoseEstimate().getHeading()) < closestTempValAngle){
                closestTempValAngle = robot.drive.getPoseEstimate().getHeading();
                closestAngle = i;
            }
        }

        PoseTransfer.idealGridCoordX = closestX;
        PoseTransfer.idealGridCoordY = closestY;
        PoseTransfer.idealGridAngle = closestAngle;

        robot.pause(30);


    }

}
