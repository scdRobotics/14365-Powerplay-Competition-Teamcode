package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "DeliveryTeleopTwoElectricBoogaloo", group = "TeleOp")
public class _DeliveryTeleopTwoElectricBoogaloo extends LinearOpMode {

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

        double slow = 1;

        double slidePos = PoseTransfer.slidePos;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;




        robot.drive.setPoseEstimate(PoseTransfer.currentPose);




        robot.delivery.initEncodersNoReset();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high


        waitForStart();


        while(!isStopRequested() && opModeIsActive()){

            /*


            ----- DRIVER 1 (DRIVETRAIN) -----


             */


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

                    robot.sensors.setLEDState(Sensors.LED_STATE.SEMI_AUTO);

                    break;


                case MANUAL:

                    //TODO: ADD SOME FORM OF SEMI_AUTO FUNCTION, PROBABLY FOR CYCLING ON ONE POLE

                    telemetry.addData("Ideal X Coord Grid: ", idealGridCoordX);
                    telemetry.addData("Ideal Y Coord Grid: ", idealGridCoordY);
                    telemetry.addData("Ideal Coord X: ", validRobotPosConversion[idealGridCoordX]);
                    telemetry.addData("Ideal Coord Y: ", validRobotPosConversion[idealGridCoordY]);
                    telemetry.addData("Ideal Heading: ", idealGridAngle);
                    telemetry.addData("Gamepad 1 Left Stick X: ", gamepad1.left_stick_x);
                    telemetry.addData("Gamepad 1 Left Stick Y: ", gamepad1.left_stick_y);
                    telemetry.update();


                    if(gamepad1.left_bumper) {
                        slow = 8;
                    }
                    else {
                        slow = 1;
                    }

                    //TODO: FIND OUT WHERE /SLOW GOES & TEST
                    //TODO: HAVE CHECK AGAINST IMU FOR IF/WHEN "GET HEADING" IS NOT ACCURATE
                    Pose2d poseEstimate = robot.drive.getPoseEstimate();
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

                    robot.drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX()/slow,
                                    input.getY()/slow,
                                    -gamepad1.right_stick_x/slow
                            )
                    );

                    robot.drive.update();


                    robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);


                    break;


            }



            /*


            ----- DRIVER 2 (SLIDE) -----


             */


            //TODO: ADD LED-ASSISTED POLE DROPOFF. WILL LIKELY BEHAVE INCREDIBLY SIMILARLY TO AUTONOMOUS SKEW CHECK, JUST SLIGHTLY DIFFERENT CONSTANTS. ONLY ACTIVATE IF SLIDE IS WITHIN DROPPING RANGES. ELSE, JUST USE DEFAULT. EXAMPLE CODE FOR LED CONTROL:
            robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
            if(slidePos>1250){
                /*
                if(isSkew()){
                    robot.sensors.setLEDState(Sensors.LED_STATE.POLE_BAD);
                }
                else{
                    robot.sensors.setLEDState(Sensors.LED_STATE.POLE_GOOD);
                }
                 */
            }



            if(gamepad2.dpad_up && !dpadUpHeld){
                slidePosIdx++;
                dpadUpHeld=true;

                slidePos = robot.delivery.slideIdxToEncoderVal(slidePosIdx);
            }

            else if(gamepad2.dpad_down && !dpadDownHeld){
                slidePosIdx--;
                dpadDownHeld=true;

                slidePos = robot.delivery.slideIdxToEncoderVal(slidePosIdx);
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


            robot.delivery.runSlide((int) (slidePos), 0.9);


            if(gamepad2.a){
                robot.delivery.runGripper(0.225);
            }
            else{
                robot.delivery.runGripper(0);
            }

            if(gamepad2.y){ //Reset button
                slidePos=0;
                slidePosIdx=0;
                robot.delivery.resetEncoders();
            }



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


                    /*if(gamepad1.y){
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
                    }*/
