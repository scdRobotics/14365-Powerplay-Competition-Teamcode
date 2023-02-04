package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RED_RIGHT_AUTO", group="Autonomous")
public class _RED_RIGHT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{


        initAuto();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(-START_X, -START_Y, Math.toRadians(START_ANG-180));

        waitForStart();

        int park = robot.vision.readAprilTagCamera2() + 1;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        robot.vision.runAprilTag(false);

        boolean robotDetected = false;

        boolean trajectorySkewFirst = false;

        boolean trajectorySkewSecond = false;

        boolean trajectorySkewThird = false;

        TrajectorySequence firstApproach = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .lineTo(new Vector2d(-I_APPROACH_X, -I_APPROACH_Y))

                .build();

        TrajectorySequence idealTrajectory = robot.drive.trajectorySequenceBuilder(firstApproach.end())

                .lineTo(new Vector2d(-I_DROP_X, -I_DROP_Y))

                //TODO: ADD FIRST TRAJECTORY "INTEGRITY CHECK" HERE

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(-I_BACK_POLE_X, I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG - (180 + 1e-6)))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(-I_PKUP_X, -I_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(-I_PKUP_BKUP_X, -I_PKUP_BKUP_Y))

                .lineTo(new Vector2d(-II_APPROACH_X, -II_APPROACH_Y))

                .turn(Math.toRadians(-II_APPROACH_TURN))

                .lineTo(new Vector2d(-II_DROP_X, -II_DROP_Y))

                //TODO: ADD FIRST TRAJECTORY "INTEGRITY CHECK" HERE

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(-II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(-II_BACK_POLE_X, -II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG - 180))) //Need to make sure this doesn't cause odo wheels to go on ground junction



                .lineTo(new Vector2d(-II_PKUP_X, -II_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(-II_PKUP_BKUP_X, -II_PKUP_BKUP_Y))

                .waitSeconds(STACK_WAIT_UP)


                .lineTo(new Vector2d(-III_APPROACH_X, -III_APPROACH_Y))

                .turn(Math.toRadians(-III_APPROACH_TURN))


                .lineTo(new Vector2d(-III_DROP_X, -III_DROP_Y))

                //TODO: ADD FIRST TRAJECTORY "INTEGRITY CHECK" HERE

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(-III_BACK_POLE_X, -III_BACK_POLE_Y, Math.toRadians(III_BACK_POLE_ANG - 180))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .build();


        double sensorReadout;

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

            sensorReadout = robot.sensors.getRightDist();

            if((sensorReadout<COLLISION_AVOIDANCE_UPPER_LIMIT && sensorReadout>COLLISION_AVOIDANCE_LOWER_LIMIT)){ //Meaning a robot is approaching the same direction
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


        }




        else {


            robot.drive.followTrajectorySequenceAsync(idealTrajectory);
            while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

                if(trajectorySkewFirst || trajectorySkewSecond || trajectorySkewThird){ //Meaning we sense we are off of the trajectory by a significant margin. One for each pole drop
                    robot.drive.breakFollowing();
                    robot.drive.setDrivePower(new Pose2d());
                    break;
                }

                // Update drive localization
                robot.drive.update();

            }

            //TODO: ADD "LIVE-BUILT" TRAJECTORIES HERE IN CASE OF SKEW

            //TODO: ADD PARKING AND POSETRANSFER PROCEDURES

            if(park==2){
                TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(PARK_II_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkTwo);
            }
            else if(park == 3){
                TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(RIGHT_PARK_III_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkThree);
            }
            else{
                TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(RIGHT_PARK_I_X, -PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkOne);
            }

        }

        PoseTransfer.currentPose = robot.drive.getPoseEstimate();
        PoseTransfer.slidePos = robot.delivery.getSlidePos();

        robot.pause(30);


    }

}
