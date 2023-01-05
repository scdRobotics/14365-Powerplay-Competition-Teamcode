package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="BLUE_RIGHT_AUTO", group="Autonomous")
public class BLUE_RIGHT_AUTO extends LinearOpMode {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Vision vision = robot.vision;
        Delivery delivery = robot.delivery;
        Sensors sensors = robot.sensors;

        delivery.initEncoders();

        vision.activateAprilTagYellowPipelineCamera1();

        robot.drive.setPoseEstimate(new Pose2d(0,0,0));

        TrajectorySequence approachPole = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    delivery.slideHigh();

                    telemetry.addData("Approach Pole Complete! ", "");
                    telemetry.update();



                })

                .lineTo(new Vector2d(55, 0))

                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(45))) //TODO: 315?


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

            if(sensors.getFrontDist()<50 && sensors.getFrontDist()>10){ //Meaning a robot is approaching the same direction
                robot.drive.breakFollowing();
                robot.drive.setDrivePower(new Pose2d());
                robotDetected=true;
            }

            // Update drive localization
            robot.drive.update();

        }

        if(robotDetected){
            //Run alt version of program (go for middle 5 point since our partner doesn't-- requires accurate scouting)
            TrajectorySequence altTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: See how accurate pose estimate is after stopping async trajectory

                    //TODO: Fill out trajectory sequence, I can't even estimate values/paths since I don't know where the localization pose starts from and don't want to figure out ideal pathing until I have the physical field in front of me

                    .build();



        }
        else{
            //Continue with normal version
            double dTheta = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole = robot.drive.trajectorySequenceBuilder(approachPole.end())
                    .turn(dTheta)
                    .build();
            robot.drive.followTrajectorySequence(turnToPole);

            double distToPole = sensors.getFrontDist();
            TrajectorySequence dropPolePickupNewCone = robot.drive.trajectorySequenceBuilder(turnToPole.end())
                    .forward(distToPole-0.5)

                    //TODO: Program trajectory for picking up new cone and returning to same spot as approachPole.end()

                    .build();
            robot.drive.followTrajectorySequence(dropPolePickupNewCone);

            double dTheta2 = vision.findClosePoleDTheta();
            TrajectorySequence turnToPole2 = robot.drive.trajectorySequenceBuilder(dropPolePickupNewCone.end())
                    .turn(dTheta2)
                    .build();
            robot.drive.followTrajectorySequence(turnToPole2);

            double distToPole2 = sensors.getFrontDist();
            TrajectorySequence dropPole = robot.drive.trajectorySequenceBuilder(turnToPole2.end())
                    .forward(distToPole2-0.5)
                    .build();
            robot.drive.followTrajectorySequence(dropPole);

            if(park==2){
                TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(dropPole.end())

                        //TODO: Program trajectory for parking in designated slot

                        .build();
                robot.drive.followTrajectorySequence(park2);
            }
            else if(park==3){
                TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(dropPole.end())

                        //TODO: Program trajectory for parking in designated slot

                        .build();
                robot.drive.followTrajectorySequence(park3);
            }
            else{
                TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(dropPole.end())

                        //TODO: Program trajectory for parking in designated slot

                        .build();
                robot.drive.followTrajectorySequence(park1);
            }

        }






        robot.pause(10);


    }

}
