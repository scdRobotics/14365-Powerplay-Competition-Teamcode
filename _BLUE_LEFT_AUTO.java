package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BLUE_LEFT_AUTO", group="Autonomous")
public class _BLUE_LEFT_AUTO extends AUTO_PRIME {

    @Override
    public void runOpMode() throws InterruptedException{

        PoseTransfer.isBlue=true;

        initAuto();
        robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

        // https://learnroadrunner.com/assets/img/field-w-axes-half.cf636a7c.jpg

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANG));

        boolean robotDetected = false;



        TrajectorySequence firstApproach = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .lineTo(new Vector2d(I_APPROACH_X, I_APPROACH_Y))

                .build();

        TrajectorySequence idealTrajectory = robot.drive.trajectorySequenceBuilder(firstApproach.end())

                .turn(Math.toRadians(I_APPROACH_TURN))

                .lineTo(new Vector2d(I_DROP_X, I_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    trajectorySkewFirst = isSkew(robot.drive.getPoseEstimate().getHeading(), robot.vision.findClosePoleDTheta(), robot.sensors.getFrontDist(), (robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/Math.sin(robot.drive.getPoseEstimate().getHeading() - Math.toRadians(180)), robot.vision.findClosePoleDist());
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(I_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(I_BACK_POLE_X, I_BACK_POLE_Y, Math.toRadians(I_BACK_POLE_ANG - 1e-6))) //Need to make sure this doesn't cause odo wheels to go on ground junction

                .lineTo(new Vector2d(I_PKUP_X, I_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(I_PKUP_BKUP_X, I_PKUP_BKUP_Y))

                .lineTo(new Vector2d(II_APPROACH_X, II_APPROACH_Y))

                .turn(Math.toRadians(-II_APPROACH_TURN))

                .lineTo(new Vector2d(II_DROP_X, II_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    trajectorySkewSecond = isSkew(robot.drive.getPoseEstimate().getHeading(), robot.vision.findClosePoleDTheta(), robot.sensors.getFrontDist(), (robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/Math.sin(robot.drive.getPoseEstimate().getHeading() - Math.toRadians(180)), robot.vision.findClosePoleDist());
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(II_BACK_POLE_X, II_BACK_POLE_Y, Math.toRadians(II_BACK_POLE_ANG))) //Need to make sure this doesn't cause odo wheels to go on ground junction



                .lineTo(new Vector2d(II_PKUP_X, II_PKUP_Y))

                .waitSeconds(STACK_WAIT_GRAB)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.closeGripper();
                    robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(STACK_WAIT_UP)

                //May need to swap these two?? Maybe, play with it a little

                .lineTo(new Vector2d(II_PKUP_BKUP_X, II_PKUP_BKUP_Y))

                .waitSeconds(STACK_WAIT_UP)


                .lineTo(new Vector2d(III_APPROACH_X, III_APPROACH_Y))

                .turn(Math.toRadians(-III_APPROACH_TURN))


                .lineTo(new Vector2d(III_DROP_X, III_DROP_Y))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    trajectorySkewThird = isSkew(robot.drive.getPoseEstimate().getHeading(), robot.vision.findClosePoleDTheta(), robot.sensors.getFrontDist(), (robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/Math.sin(robot.drive.getPoseEstimate().getHeading() - Math.toRadians(180)), robot.vision.findClosePoleDist());
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.slideControl(II_CONE_STACK_PICKUP_HEIGHT, SLIDE_POWER);
                })

                .waitSeconds(POLE_WAIT_DROP)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.delivery.openGripper();
                })

                .waitSeconds(POLE_WAIT_RELEASE)

                .lineToLinearHeading(new Pose2d(III_BACK_POLE_X, III_BACK_POLE_Y, Math.toRadians(III_BACK_POLE_ANG))) //Need to make sure this doesn't cause odo wheels to go on ground junction

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
                    break;
                }

                // Update drive localization
                robot.drive.update();

            }

            //TODO: ADD "LIVE-BUILT" TRAJECTORIES HERE IN CASE OF SKEW


            if(park==2){
                TrajectorySequence parkTwo = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(PARK_II_X, PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkTwo);
            }
            else if(park == 3){
                TrajectorySequence parkThree = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(LEFT_PARK_I_X, PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkThree);
            }
            else{
                TrajectorySequence parkOne = robot.drive.trajectorySequenceBuilder(startPose)

                        .lineTo(new Vector2d(LEFT_PARK_I_X, PARK_Y))

                        .build();

                robot.drive.followTrajectorySequence(parkOne);
            }

        }

        PoseTransfer.currentPose = robot.drive.getPoseEstimate();
        PoseTransfer.slidePos = robot.delivery.getSlidePos();

        //Find actual closest coord grid values in case park goes wrong, and also prevents a code block for each block case
        //TODO: NEEDS TESTING TO ENSURE IT ACTUALLY WORKS PROPERLY

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

    //TODO: MAY NEED TO SWAP NEGATIVE AND ANGLE VALUES HERE AND THERE. REVIEW WITH OTHERS BECAUSE I'M TIRED AND GO INSANE THE MORE I LOOK AT IT BY MYSELF.
    //TODO: ENTIRE THING NEEDS TESTING IN EVERT POSSIBLE USE CASE. BADLY. VERY BADLY.
    //TODO: ENTIRE THING NEEDS TO BE TRANSFERRED TO OTHER AUTOS (AN ORDEAL BY ITSELF) ONCE IT'S TESTED WORKING
    public boolean isSkew(double localizerTheta, double camTheta, double frontSensorDist, double localizerDist, double camDist){

        /*
        TELEMETRY CALLS
         */

        telemetry.addData("-----Skew Passthrough #-----", SKEW_COUNT);

        telemetry.addData("Localizer Theta: ", localizerTheta);
        telemetry.addData("Cam Theta: ", camTheta);
        telemetry.addData("Front Sensor Dist: ", frontSensorDist);
        telemetry.addData("Localizer Dist: ", localizerDist);
        telemetry.addData("Cam Dist: ", camDist);

        telemetry.addData("Is Localizer Theta Accurate? ", isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA));
        telemetry.addData("Is Camera Theta Accurate? ", isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA));
        telemetry.addData("Is Front Sensor Dist Accurate? ", isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));
        telemetry.addData("Is Localizer Dist Accurate? ", isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));
        telemetry.addData("Is Webcam Dist Accurate? ", isEqual(camDist, WEBCAM_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));

        telemetry.update();


        /*
        MAIN CHECK-- IS EVERYTHING GOOD?
         */
        if(
                isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)
                && isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)
                && isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
                && isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
                && isEqual(camDist, WEBCAM_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
        ){
          /*
          IF EVERYTHING IS GOOD, NO SKEW! ELSE, CONTINUE
           */
          SKEW_COUNT=0;
          return false;
        }

        else{

            if(!isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)){ //IF CAM THETA IS NOT WHAT WE EXPECT...
                camThetaAccurate=false; //No longer flagged as accurate
                camTheta = robot.vision.findClosePoleDTheta(); //Update camera in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                camThetaAccurate=true; //Flagged as accurate
            }

            if(!isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)){ //IF LOCALIZER THETA IS NOT WHAT WE EXPECT...
                localizerThetaAccurate=false; //No longer flagged as accurate
                if(camThetaAccurate){ //If camera theta is accurate, we have a reference point! (Albeit, one with a good amount of noise we maybe shouldn't rely on... needs testing)
                    robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), camTheta + BLUE_LEFT_IDEAL_THETA)); //Update localizer estimate based on camTheta readout
                }
            }
            else{ //IF IT IS WHAT WE EXPECT...
                localizerThetaAccurate=true; //Flagged as accurate
            }

            if(!isEqual(camDist, Math.toRadians(WEBCAM_DIST_ACCEPTABLE_RANGE), IDEAL_DIST)){ //IF CAMERA DIST IS NOT WHAT WE EXPECT...
                camDistAccurate=false; //No longer flagged as accurate
                camDist = robot.vision.findClosePoleDist(); //Update camera in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                camDistAccurate=true; //Flagged as accurate
            }

            if(!isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)){ //IF FRONT SENSOR DIST IS NOT WHAT WE EXPECT...
                frontSensorDistAccurate=false; //No longer flagged as accurate
                frontSensorDist = robot.sensors.getFrontDist(); //Update sensor in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                frontSensorDistAccurate=true; //Flagged as accurate
            }

            if(!isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)){ //IF ODOMETRY DIST IS NOT WHAT WE EXPECT... (this one's a doozy)

                if(localizerThetaAccurate){ //IF LOCALIZER THETA (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                    double cos = Math.cos(localizerTheta - Math.toRadians(180)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/ cos, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated X is outside tolerance... (ask Logan about math- it's just trig triangles)

                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *frontSensorDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on sensor calculated X dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *camDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }


                    double sin = Math.sin(localizerTheta - Math.toRadians(180)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getY())/ sin, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated Y is outside tolerance... (ask Logan about math- it's just trig triangles)
                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *frontSensorDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer Y estimate based on sensor calculated Y dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *camDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }
                }


                else if(camThetaAccurate){ //IF CAMERA THETA (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                    double cos = Math.cos(camTheta + Math.toRadians(45)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/ cos, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated X is outside tolerance... (ask Logan about math- it's just trig triangles)

                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *frontSensorDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on sensor calculated X dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *camDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }


                    double sin = Math.sin(camTheta - Math.toRadians(45)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getY())/ sin, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated Y is outside tolerance... (ask Logan about math- it's just trig triangles)
                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *frontSensorDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer Y estimate based on sensor calculated Y dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *camDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }
                }


                else{
                    //Can't do anything else :( too much is messed up and totally wrong. hope it fixes itself on a recursive call. this should never happen since we should always have at least the webcams viewing the pole but, you never know.
                }

                localizerDistAccurate=false; //No longer flagged as accurate

            }else{
                localizerDistAccurate=true; //Flagged as accurate
            }

            if(localizerDistAccurate && localizerThetaAccurate){
                //Just localizer is good enough to act on-- no skew here folks, juuust potentially faulty sensors!
                return false;
            }

            if(SKEW_COUNT<4){ //IF WE HAVENT RECURSIVE CALLED 4 OR MORE TIMES...
                SKEW_COUNT++; //Increase count
                isSkew(robot.drive.getPoseEstimate().getHeading(), camTheta, robot.drive.getPoseEstimate().getY()/Math.sin(camTheta - Math.toRadians(45)), localizerDist, camDist); //Recursive function yay! This just goes back and checks sensors again after aalll refreshes in logic above have occured...
            }

            robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED); //We did all recursive calls and still, something's wrong. Big uh oh. Just go to live built trajectories and hope robot can mostly fix itself, probably. NOTE: ITS EXTREMELY RARE FOR THIS TO HAPPEN SINCE IT MEANS EVERY SINGLE SENSOR HAS IN SOME WAY MESSED UP. FUNCTIONALLY, WE WILL NEVER REACH THIS-- BUT BETTER TO HOPE "IT'LL FIGURE IT OUT WITH LOCALIZER ADJUSTMENTS" THAN HAVE IT CRASH HERE
            return true; //We skewed :(
        }

    }

    public boolean isEqual(double a, double aRange, double b, double bRange){
        return (a-aRange <= b+bRange) && (b-bRange <= a+aRange);
    }

    /*public boolean isEqual(double a, double aRange, double b){
        //return (a-aRange <= b) && (b <= a+aRange);
    }*/

    public boolean isEqual (double x, double delta, double a) //X = sensor input, A = ideal input, delta = range/2
    {
        return Math.abs(x-a) < (delta/2);
    }


}
