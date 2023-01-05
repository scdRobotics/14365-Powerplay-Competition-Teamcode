package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(name = "GridTeleOp", group = "TeleOp")
public class BLUE_GRID_TELEOP extends LinearOpMode {

    public enum DRIVE_MODE {
        AUTO,
        MANUAL
    }

    DRIVE_MODE drive_mode = DRIVE_MODE.AUTO;

    @Override
    public void runOpMode(){

        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Delivery delivery = robot.delivery;

        double slow = 1;

        double slidePos = 0;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;

        waitForStart();

        delivery.initEncoders();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high

        boolean firstSwitchLoop = true;




        //Will have to (somehow) inherit these from auto
        int currentGridX = 0;
        int currentGridY = 0;
        int currentGridOrientation = 0;


        //Will help interface between Grid Values and Roadrunner Values
        /*
         ONE TILE LENGTH = 24 INCHES.
         SO, A PERFECT CURRENTGRIDX/CURRENTGRIDY = 12 INCHES (middle of tile)
        */
        /*int currentCoordX = 0;
        int currentCoordY = 0;*/

        boolean gamepadAHeld = false;







        while(!isStopRequested() && opModeIsActive()){



            /*

                            (0)--------------(Y)--------------(5)
            (0,0)
                            /-----------------------------------\                    (0)
                            |     *     *     *     *     *     |                     |
                            |*****o*****o*****o*****o*****o*****|                     |
                            |     *     *     *     *     *     |                     |                             1
                            |*****O*****o*****O*****o*****O*****|                     |                             ^
                            |     *     *     *     *     *     |                     |                             |
              Blue Side     |*****O*****o*****O*****o*****O*****|     Red Side       (X)                2 <-- (Orientation) --> 0
                            |     *     *     *     *     *     |                     |                             |
                            |*****o*****o*****o*****o*****o*****|                     |                             ∨
                            |     *     *     *     *     *     |                     |                             3
                            |*****O*****o*****O*****o*****O*****|                     |
                            |     *     *     *     *     *     |                     |
                            \-----------------------------------/                    (5)
                                                                         (5,5)

             */


            switch(drive_mode){
                case AUTO:
                    if(gamepad1.a && !gamepadAHeld){

                        robot.drive.breakFollowing();
                        robot.drive.setDrivePower(new Pose2d());

                        gamepadAHeld = true;
                        firstSwitchLoop = true;

                        drive_mode=DRIVE_MODE.MANUAL;

                    }

                    if(firstSwitchLoop){
                        robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        //Should also update current robot pose here (somehow...)
                    }

                    firstSwitchLoop = false;

                    //May change to be driver oriented rather than field oriented- AKA, integrated into the big ugly switch case below. Shouldn't be necessary, this seems fairly intuitive with some practice
                    if(gamepad1.right_stick_x>0.1){
                        currentGridOrientation++;
                    }
                    else if(gamepad1.right_stick_x<-0.1){
                        currentGridOrientation--;
                    }

                    switch(currentGridOrientation){
                        case 0 & 2:
                            if(gamepad1.dpad_up){
                                currentGridY++;
                            }
                            else if(gamepad1.dpad_down){
                                currentGridY--;
                            }

                            if(gamepad1.dpad_left){
                                currentGridX--;
                            }
                            else if(gamepad1.dpad_right){
                                currentGridX++;
                            }

                            break;

                        case 1 & 3:
                            if(gamepad1.dpad_up){
                                currentGridX--;
                            }
                            else if(gamepad1.dpad_down){
                                currentGridX++;
                            }

                            if(gamepad1.dpad_left){
                                currentGridY--;
                            }
                            else if(gamepad1.dpad_right){
                                currentGridY++;
                            }
                            break;

                    }

                    if(currentGridY<0){
                        currentGridY=3;
                    }
                    else if(currentGridY>3){
                        currentGridY=0;
                    }

                    if(currentGridX<0){
                        currentGridX=3;
                    }
                    else if(currentGridX>3){
                        currentGridX=0;
                    }

                    TrajectorySequence grid = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: Make sure we have some way to make getPoseEstimate accurate between switching modes and between auto & teleop

                            .lineTo(new Vector2d(gridToAbsolute(currentGridX), gridToAbsolute(currentGridY)))
                            .turn(orientationToRadians(currentGridOrientation))

                            .build();

                    robot.drive.followTrajectorySequenceAsync(grid);

                    robot.drive.update();

                    if(!robot.drive.isBusy()){
                        if(gamepad1.left_stick_x>0.2 && -gamepad1.left_stick_y>0.2){ //Front right

                        }
                        else if(gamepad1.left_stick_x>0.2 && -gamepad1.left_stick_y<-0.2){ //Back Right

                        }
                        else if(gamepad1.left_stick_x<-0.2 && -gamepad1.left_stick_y<-0.2){ //Back Left

                        }
                        else if(gamepad1.left_stick_x<-0.2 && -gamepad1.left_stick_y>0.2){ //Front Left

                        }
                    }

                    break;



                case MANUAL:
                    if(gamepad1.a && !gamepadAHeld){

                        robot.drive.setMotorPowers(0,0,0,0);

                        gamepadAHeld = true;
                        firstSwitchLoop = true;

                        drive_mode=DRIVE_MODE.AUTO;
                    }

                    if(firstSwitchLoop){

                    }

                    firstSwitchLoop = false;

                    robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Ideally, would only declare first time case was switched. Look into how to accomplish this

                    double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;

                    if(gamepad1.left_bumper && rx>0.1){
                        slow=5;
                    }
                    else if(gamepad1.left_bumper && (y>0.1 || x>0.1)){
                        slow=2;
                    }
                    else{
                        slow=1;
                    }

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    if(y<0.075 || x<0.0825 || rx<0.075){ //Account for potential joystick drift
                        /*robot.frontLeftM.setPower(frontLeftPower/slow);
                        robot.backLeftM.setPower(backLeftPower/slow);
                        robot.frontRightM.setPower(frontRightPower/slow);
                        robot.backRightM.setPower(backRightPower/slow);*/
                        robot.drive.setMotorPowers(frontLeftPower/slow, backLeftPower/slow, backRightPower/slow, frontRightPower/slow);
                    }
                    else{
                        /*robot.frontLeftM.setPower(0);
                        robot.frontRightM.setPower(0);
                        robot.backLeftM.setPower(0);
                        robot.backRightM.setPower(0);
                         */
                        robot.drive.setMotorPowers(0,0,0,0);
                    }

                    break;



            }

            if(!gamepad1.a){
                gamepadAHeld=false;
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




            delivery.getEncoderValues();



        }

    }

    public double gridToAbsolute(int coord){
        return (coord * 24) + 12;
    }

    public double orientationToRadians(int orientation){
        return Math.toRadians(orientation*90);
    }
}
