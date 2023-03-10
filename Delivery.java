package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//Most imports are automatically handled by Android Studio as you program

public class Delivery extends Subsystem {
    private final DcMotorEx slide;
    private final Servo gripper;

    //"Constructor" object for Delivery
    public Delivery(DcMotorEx slide, Servo gripper, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer);
        this.slide=slide;
        this.gripper=gripper;
    }

    public void getEncoderValues(){
        telemetry.addData("Slide Encoder Position: ", slide.getCurrentPosition());
        telemetry.update();
    }

    public double getSlidePos(){
        return slide.getCurrentPosition();
    }

    public void initEncoders(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setTargetPositionTolerance(50);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initEncodersNoReset(){
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setTargetPositionTolerance(50);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runSlide(int pos, double power){
        slide.setTargetPosition(pos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }

    /*public void stopSlide(){
        slide.setPower(0);
    }*/

    public int slideIdxToEncoderVal(int idx){


        if(idx==0){
            return 0;
        }
        else if(idx==1){
            return 1776;
        }
        else if(idx==2){
            return 2900;
        }
        else if(idx==3){
            return 4000;
        }

        return 0;
    }

    public void slideControl(int pos, double pow){
        slide.setTargetPosition(pos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(pow);
    }

    public void resetEncoders(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runGripper(double pos){
        gripper.setPosition(pos);
    }

    public void openGripper(){
        gripper.setPosition(1);
    }


    public void closeGripper(){
        gripper.setPosition(0);
    }


}
