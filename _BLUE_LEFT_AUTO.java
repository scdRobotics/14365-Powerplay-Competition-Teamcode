package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BLUE_LEFT_AUTO", group="Autonomous")
public class _BLUE_LEFT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{


        initAuto();

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));

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

                .lineTo(new Vector2d(FIRST_APPROACH_X, FIRST_APPROACH_Y))

                .build();

        TrajectorySequence idealTrajectory = robot.drive.trajectorySequenceBuilder(firstApproach.end())

                .lineTo(new Vector2d(FIRST_DROP_X, FIRST_DROP_Y))

                .waitSeconds(POLE_WAIT_DROP)

                //TODO: ADD FIRST TRAJECTORY "INTEGRITY CHECK" HERE

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.delivery.openGripper();

                })

                .lineToLinearHeading(new Pose2d(FIRST_BACKOFF_POLE_X, FIRST_BACKOFF_POLE_Y, FIRST_BACKOFF_POLE_ANGLE)) //Need to make sure this doesn't cause odo wheels to go on ground junction

                //Next: Go to cone stack



                .build();


        double sensorReadout;

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

            sensorReadout = robot.sensors.getLeftDist();

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

        }

        PoseTransfer.currentPose = robot.drive.getPoseEstimate();
        PoseTransfer.slidePos = robot.delivery.getSlidePos();

        robot.pause(30);


    }

}
