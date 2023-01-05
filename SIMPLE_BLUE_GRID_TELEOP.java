package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(name = "SIMPLE_BLUE_GRID_TELEOP", group = "TeleOp")
public class SIMPLE_BLUE_GRID_TELEOP extends LinearOpMode {

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

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if(gamepad1.left_bumper && (rx>0.1 || rx<-0.1)){
                slow=5;
            }
            else if(gamepad1.left_bumper && (y>0.1 || x>0.1 || x<-0.1 || y<-0.1)){
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

            if(!robot.drive.isBusy()){
                if(y<0.075 || x<0.0825 || rx<0.075){
                    robot.drive.setMotorPowers(frontLeftPower/slow, backLeftPower/slow, backRightPower/slow, frontRightPower/slow);
                }
                else{
                    robot.drive.setMotorPowers(0,0,0,0);
                }
            }


            robot.drive.update();

            TrajectorySequence forwardGrid = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: Make sure we have some way to make getPoseEstimate accurate between switching modes and between auto & teleop

                    .forward(24)

                    .build();

            TrajectorySequence backwardGrid = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: Make sure we have some way to make getPoseEstimate accurate between switching modes and between auto & teleop

                    .forward(-24)

                    .build();

            TrajectorySequence strafeRightGrid = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: Make sure we have some way to make getPoseEstimate accurate between switching modes and between auto & teleop

                    .strafeRight(24)

                    .build();

            TrajectorySequence strafeLeftGrid = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) //TODO: Make sure we have some way to make getPoseEstimate accurate between switching modes and between auto & teleop

                    .strafeLeft(24)

                    .build();

            TrajectorySequence turnLeft = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                    .turn(Math.toRadians(robot.drive.getPoseEstimate().getHeading()-90))

                    .build();

            TrajectorySequence turnRight = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                    .turn(Math.toRadians(robot.drive.getPoseEstimate().getHeading()+90))

                    .build();





            if(gamepad1.dpad_up){
                robot.drive.followTrajectorySequence(forwardGrid);
            }
            else if(gamepad1.dpad_right){
                robot.drive.followTrajectorySequence(strafeRightGrid);
            }
            else if(gamepad1.dpad_left){
                robot.drive.followTrajectorySequence(strafeLeftGrid);
            }
            else if(gamepad1.dpad_down){
                robot.drive.followTrajectorySequence(backwardGrid);
            }

            if(gamepad1.b){
                robot.drive.followTrajectorySequence(turnRight);
            }
            else if(gamepad1.x){
                robot.drive.followTrajectorySequence(turnLeft);
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
