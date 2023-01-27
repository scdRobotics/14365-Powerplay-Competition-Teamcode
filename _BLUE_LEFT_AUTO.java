package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALIGN_POLE_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_ALIGN_POLE_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_BACK_OFF_FROM_POLE_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_BACK_OFF_FROM_POLE_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_CONE_STACK_TURN_TOWARD_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_1_X_LEFT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_1_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_2_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_2_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_3_X_LEFT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_PARK_3_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_POLE_DEFAULT_TRAVEL_DIST;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_POLE_DISTANCE_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_START_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_START_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.ALT_START_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.CONE_STACK_TURN_TOWARD_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.CONE_STACK_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.CONE_STACK_X_BACKUP;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.CONE_STACK_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.CONE_STACK_Y_BACKUP;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_ALIGN_POLE_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_ALIGN_POLE_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_APPROACH_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_APPROACH_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_BACK_OFF_FROM_POLE_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_BACK_OFF_FROM_POLE_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_POLE_DEFAULT_TRAVEL_DIST;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_POLE_DISTANCE_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_ROBOT_DISTANCE_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.FIRST_ROBOT_DISTANCE_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_1_X_LEFT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_1_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_2_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_2_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_3_X_LEFT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.PARK_3_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.POLE_WAIT_DROP;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.POLE_WAIT_RELEASE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_ALIGN_POLE_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_ALIGN_POLE_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_BACK_OFF_FROM_POLE_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_BACK_OFF_FROM_POLE_Y;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_POLE_DEFAULT_TRAVEL_DIST;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.SECOND_POLE_DISTANCE_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.START_ANGLE;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.START_X;
import static org.firstinspires.ftc.teamcode.AUTO_CONSTANTS.START_Y;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BLUE_LEFT_AUTO", group="Autonomous")
public class _BLUE_LEFT_AUTO extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();



                })

                .lineTo(new Vector2d(FIRST_APPROACH_X, FIRST_APPROACH_Y))



                .build();


        TrajectorySequence alignPole = robot.drive.trajectorySequenceBuilder(approachPole.end())

                .lineToLinearHeading(new Pose2d(FIRST_ALIGN_POLE_X, FIRST_ALIGN_POLE_Y, Math.toRadians(ALIGN_POLE_ANGLE - 90)))

                .build();

        delivery.closeGripper();

        waitForStart();

        int park = vision.readAprilTagCamera1() + 1;

        telemetry.addData("April Tag Detected: ", park);
        telemetry.update();

        vision.runAprilTag(false);

        boolean robotDetected = false;

        robot.drive.followTrajectorySequenceAsync(approachPole);

        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){ //Should leave loop when async function is done or robot is detected

            telemetry.addData("Front Left: ", sensors.getFrontLeftDist());
            telemetry.addData("Front Right: ", sensors.getFrontRightDist());
            telemetry.update();

            if((sensors.getFrontRightDist()<FIRST_ROBOT_DISTANCE_UPPER_LIMIT && sensors.getFrontRightDist()>FIRST_ROBOT_DISTANCE_LOWER_LIMIT) || (sensors.getFrontLeftDist()<FIRST_ROBOT_DISTANCE_UPPER_LIMIT && sensors.getFrontLeftDist()>FIRST_ROBOT_DISTANCE_LOWER_LIMIT)){ //Meaning a robot is approaching the same direction
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

                    .lineToLinearHeading(new Pose2d(ALT_START_X, ALT_START_Y, Math.toRadians(ALT_START_ANGLE)))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.slideMed();

                        telemetry.addData("Alt Traj Complete! ", "");
                        telemetry.update();



                    })

                    .turn(Math.toRadians(-ALT_ALIGN_POLE_ANGLE))

                    .build();

            robot.drive.followTrajectorySequence(altTraj);


            //double dTheta = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(altTraj.end())
                    //.turn(dTheta)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist() - ALT_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
            if(distToPole>ALT_POLE_DISTANCE_UPPER_LIMIT){
                distToPole=ALT_POLE_DEFAULT_TRAVEL_DIST;
            }

            TrajectorySequence dropPoleMid = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(ALT_BACK_OFF_FROM_POLE_X, ALT_BACK_OFF_FROM_POLE_Y))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .turn(Math.toRadians(ALT_CONE_STACK_TURN_TOWARD_ANGLE))

                    .build();

            robot.drive.followTrajectorySequence(dropPoleMid);

            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(ALT_PARK_2_X, ALT_PARK_2_Y))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.idealGridCoordX=4;
                PoseTransfer.idealGridCoordY=4;
                PoseTransfer.idealGridAngle=0;


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(ALT_PARK_3_X_LEFT, ALT_PARK_3_Y))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.idealGridCoordX=3;
                PoseTransfer.idealGridCoordY=4;
                PoseTransfer.idealGridAngle=0;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropPoleMid.end())

                        .lineToConstantHeading(new Vector2d(ALT_PARK_1_X_LEFT, ALT_PARK_1_Y))

                        .build();

                robot.drive.followTrajectorySequence(park1);

                PoseTransfer.idealGridCoordX=5;
                PoseTransfer.idealGridCoordY=4;
                PoseTransfer.idealGridAngle=0;


            }


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

            double distToPole = sensors.getFrontDist() - FIRST_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
            telemetry.addData("Front Dist: ", sensors.getFrontDist());
            telemetry.update();
            if(distToPole>FIRST_POLE_DISTANCE_UPPER_LIMIT){
                distToPole=FIRST_POLE_DEFAULT_TRAVEL_DIST;
            }
            TrajectorySequence dropPolePickupNewCone = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole)

                    .waitSeconds(POLE_WAIT_DROP)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .waitSeconds(POLE_WAIT_RELEASE)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(FIRST_BACK_OFF_FROM_POLE_X, FIRST_BACK_OFF_FROM_POLE_Y))

                    .turn(Math.toRadians(CONE_STACK_TURN_TOWARD_ANGLE))

                    .lineToConstantHeading(new Vector2d(CONE_STACK_X, CONE_STACK_Y))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        //robot.pause(1);

                        delivery.closeGripper();

                        //robot.pause(1);



                    })

                    .lineTo(new Vector2d(CONE_STACK_X_BACKUP, CONE_STACK_Y_BACKUP))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slideHigh();
                    })

                    .waitSeconds(0.175)



                    .lineTo(new Vector2d(SECOND_ALIGN_POLE_X, SECOND_ALIGN_POLE_Y))

                    .turn(Math.toRadians(-CONE_STACK_TURN_TOWARD_ANGLE))


                    .build();



            robot.drive.followTrajectorySequence(dropPolePickupNewCone);

            //double dTheta2 = vision.findClosePoleDTheta();

            TrajectorySequence turnToPole2 = robot.drive.trajectorySequenceBuilder(dropPolePickupNewCone.end())
                    //.turn(dTheta2)
                    .turn(Math.toRadians(1))
                    .build();

            robot.drive.followTrajectorySequence(turnToPole2);

            double distToPole2 = sensors.getFrontDist() - SECOND_POLE_DISTANCE_SUBTRACTIVE_MODIFIER;
            if(distToPole2>SECOND_POLE_DISTANCE_UPPER_LIMIT){
                distToPole2=SECOND_POLE_DEFAULT_TRAVEL_DIST;
            }
            telemetry.addData("Front Dist: ", sensors.getFrontDist());
            telemetry.update();

            TrajectorySequence dropLastCone = robot.drive.trajectorySequenceBuilder(turnToPole2.end())
                    .forward(distToPole2)

                    .waitSeconds(POLE_WAIT_DROP)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        delivery.slidePickupStack();
                    })

                    .waitSeconds(POLE_WAIT_RELEASE)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        delivery.openGripper();


                    })

                    .lineToConstantHeading(new Vector2d(SECOND_BACK_OFF_FROM_POLE_X, SECOND_BACK_OFF_FROM_POLE_Y))



                    .turn(Math.toRadians(CONE_STACK_TURN_TOWARD_ANGLE))

                    .build();



            robot.drive.followTrajectorySequence(dropLastCone);




            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(PARK_2_X, PARK_2_Y))

                        .build();

                robot.drive.followTrajectorySequence(park2);

                PoseTransfer.idealGridCoordX=4;
                PoseTransfer.idealGridCoordY=3;
                PoseTransfer.idealGridAngle=0;


            }

            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(PARK_3_X_LEFT, PARK_3_Y))

                        .build();

                robot.drive.followTrajectorySequence(park3);

                PoseTransfer.idealGridCoordX=3;
                PoseTransfer.idealGridCoordY=3;
                PoseTransfer.idealGridAngle=0;


            }

            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropLastCone.end())

                        .lineToConstantHeading(new Vector2d(PARK_1_X_LEFT, PARK_1_Y))

                        .build();

                robot.drive.followTrajectorySequence(park1);

                PoseTransfer.idealGridCoordX=5;
                PoseTransfer.idealGridCoordY=3;
                PoseTransfer.idealGridAngle=0;


            }

        }

        PoseTransfer.currentPose=robot.drive.getPoseEstimate();

        robot.pause(30);


    }

}
