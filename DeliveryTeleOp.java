package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "DeliveryTeleOp", group = "TeleOp")
public class DeliveryTeleOp extends LinearOpMode {

    @Override
    public void runOpMode(){

        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, true);

        robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

        //robot.drive.setPoseEstimate(PoseTransfer.currentPose);

        double slow = 1;

        double slidePos = 0;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;

        waitForStart();

        robot.delivery.initEncoders();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high

        while(!isStopRequested()){



            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if(gamepad1.left_bumper && (rx>0.1 || rx<-0.1)){
                slow=3;
            }
            else if(gamepad1.left_bumper && (y>0.1 || x>0.1 || y<-0.1 || x<-0.1)){
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
                robot.frontLeftM.setPower(frontLeftPower/slow);
                robot.backLeftM.setPower(backLeftPower/slow);
                robot.frontRightM.setPower(frontRightPower/slow);
                robot.backRightM.setPower(backRightPower/slow);
            }
            else{
                robot.frontLeftM.setPower(0);
                robot.frontRightM.setPower(0);
                robot.backLeftM.setPower(0);
                robot.backRightM.setPower(0);
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


            if(slidePos>1250){
                robot.sensors.setLEDState(Sensors.LED_STATE.POLE_BAD);
            }
            else{
                robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
            }


            //robot.delivery.getEncoderValues();



        }

    }
}
