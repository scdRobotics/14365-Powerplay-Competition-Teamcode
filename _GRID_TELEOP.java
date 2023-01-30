package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;


@TeleOp(name = "GRID_TELEOP", group = "TeleOp")
public class _GRID_TELEOP extends LinearOpMode {

    enum MODE {
        MANUAL,
        AUTO
    }

    int idealGridCoordX = PoseTransfer.idealGridCoordX;

    int idealGridCoordY = PoseTransfer.idealGridCoordY;

    double idealGridAngle = PoseTransfer.idealGridAngle;


    MODE currentMode = MODE.AUTO;

    @Override
    public void runOpMode(){

        //0,0 = Blue right corner square
        //0,5 = Blue left corner square
        //0,5 = Red left corner square
        //5,5 = Red right corner square



        int[] validRobotPosConversion = new int[6];

        for(int i = 0; i< validRobotPosConversion.length; i++){
            validRobotPosConversion[i]= ((i*24) + 12) - 72;
        }


        Pose2d idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Delivery delivery = robot.delivery;

        double slow = 1;

        double slidePos = PoseTransfer.slidePos;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;




        robot.drive.setPoseEstimate(PoseTransfer.currentPose);




        delivery.initEncodersNoReset();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high



        TrajectorySequence goToFirstPoint = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                .lineToLinearHeading(idealPose)

                .build();


        waitForStart();

        robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.drive.followTrajectorySequenceAsync(goToFirstPoint);


        while(!isStopRequested() && opModeIsActive()){






            switch(currentMode){
                case AUTO:

                    if(gamepad1.left_stick_button){
                        robot.drive.breakFollowing();
                        currentMode = MODE.MANUAL;
                    }

                    if(!robot.drive.isBusy()){
                        robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
                        currentMode= MODE.MANUAL;
                    }

                    robot.drive.update();

                    break;


                case MANUAL:

                    telemetry.addData("Ideal X Coord Grid: ", idealGridCoordX);
                    telemetry.addData("Ideal Y Coord Grid: ", idealGridCoordY);
                    telemetry.addData("Ideal Coord X: ", validRobotPosConversion[idealGridCoordX]);
                    telemetry.addData("Ideal Coord Y: ", validRobotPosConversion[idealGridCoordY]);
                    telemetry.addData("Ideal Heading: ", idealGridAngle);
                    telemetry.addData("Gamepad 1 Left Stick X: ", gamepad1.left_stick_x);
                    telemetry.addData("Gamepad 1 Left Stick Y: ", gamepad1.left_stick_y);
                    telemetry.update();

                    Pose2d poseEstimate = robot.drive.getPoseEstimate();

                    if(gamepad1.dpad_up){

                        idealGridCoordY++;

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }
                    else if(gamepad1.dpad_right){
                        idealGridCoordX++;

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;
                    }
                    else if(gamepad1.dpad_down){
                        idealGridCoordY--;

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }
                    else if(gamepad1.dpad_left){
                        idealGridCoordX--;

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }

                    if(gamepad1.b){
                        idealGridAngle+=90;

                        checkBoundaries();

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .turn(Math.toRadians(90))
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;


                    }
                    else if(gamepad1.x){
                        idealGridAngle-=90;

                        checkBoundaries();

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .turn(Math.toRadians(-90))
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }

                    if(gamepad1.left_bumper){
                        idealPose = new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;
                    }


                    if(gamepad1.left_bumper && (gamepad1.right_stick_x>0.1 || gamepad1.right_stick_x<-0.1)){
                        slow=5;
                    }
                    else if(gamepad1.left_bumper && (-gamepad1.left_stick_y>0.1 || gamepad1.left_stick_x>0.1 || gamepad1.left_stick_x<-0.1 || -gamepad1.left_stick_y<-0.1)){
                        slow=2;
                    }
                    else{
                        slow=1;
                    }

                    if(gamepad1.a){
                        //TODO: AUTONOMOUS POLE movements (should be straightforward. Mostly copied from auto.)

                        if(gamepad1.left_stick_x>0.25 && -gamepad1.left_stick_y>0.25){
                            TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(45))
                                        .forward(10)
                                        .build();

                            robot.drive.followTrajectorySequenceAsync(traj);
                            currentMode = MODE.AUTO;
                        }
                        else if(gamepad1.left_stick_x<-0.25 && -gamepad1.left_stick_y>0.25){
                            TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                    .turn(Math.toRadians(-45))
                                    .forward(10)
                                    .build();

                            robot.drive.followTrajectorySequenceAsync(traj);
                            currentMode = MODE.AUTO;
                        }
                        else if(gamepad1.left_stick_x<-0.25 && -gamepad1.left_stick_y<-0.25){
                            TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                    .turn(Math.toRadians(-135))
                                    .forward(10)
                                    .build();

                            robot.drive.followTrajectorySequenceAsync(traj);
                            currentMode = MODE.AUTO;
                        }
                        else if(gamepad1.left_stick_x>0.25 && -gamepad1.left_stick_y<-0.25){
                            TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                    .turn(Math.toRadians(135))
                                    .forward(10)
                                    .build();

                            robot.drive.followTrajectorySequenceAsync(traj);
                            currentMode = MODE.AUTO;

                        }

                    }


                    else{
                        //MANUAL movements
                        //TODO: Figure out how to disable following/heading PID to avoid jerky manual movements
                        robot.drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y/slow,
                                        gamepad1.left_stick_x/slow,
                                        -gamepad1.right_stick_x/slow
                                )
                        );
                    }



                    if(gamepad1.y){
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
                            if(Math.abs(idealGridAngle - robot.drive.getPoseEstimate().getHeading()) < closestTempValAngle){
                                closestTempValAngle = robot.drive.getPoseEstimate().getHeading();
                                closestAngle = i;
                            }
                        }

                        idealGridCoordX = closestX;
                        idealGridCoordY = closestY;
                        idealGridAngle = closestAngle;

                        robot.drive.setPoseEstimate(new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle)));
                    }

                    robot.drive.update();


                    break;




            }









            if(gamepad2.dpad_up && !dpadUpHeld){
                slidePosIdx++;
                dpadUpHeld=true;

                slidePos = delivery.slideIdxToEncoderVal(slidePosIdx);
            }

            else if(gamepad2.dpad_down && !dpadDownHeld){
                slidePosIdx--;
                dpadDownHeld=true;

                slidePos = delivery.slideIdxToEncoderVal(slidePosIdx);
            }

            if(!gamepad2.dpad_up){
                dpadUpHeld=false;
            }

            if(!gamepad2.dpad_down){
                dpadDownHeld=false;
            }





            if(gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1){
                slidePos += -gamepad2.left_stick_y*10;
            }


            delivery.runSlide((int) (slidePos), 0.9);


            if(gamepad2.a){
                delivery.runGripper(0.225);
            }
            else{
                delivery.runGripper(0);
            }

            if(gamepad2.y){ //Reset button
                slidePos=0;
                slidePosIdx=0;
                delivery.resetEncoders();
            }




            //delivery.getEncoderValues();



        }

    }

    public void checkBoundaries(){
        if(idealGridAngle>360){
            idealGridAngle-=360;
        }
        else if(idealGridAngle<0){
            idealGridAngle+=360;
        }

        if(idealGridCoordX>5){
            idealGridCoordX=5;
        }
        else if(idealGridCoordX<0){
            idealGridCoordX=0;
        }

        if(idealGridCoordY>5){
            idealGridCoordY=5;
        }
        else if(idealGridCoordY<0){
            idealGridCoordY=0;
        }
    }

}
