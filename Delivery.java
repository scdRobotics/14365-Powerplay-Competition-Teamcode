package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//Most imports are automatically handled by Android Studio as you program

public class Delivery extends Subsystem {
    public final DcMotorEx slide; //Special Intake Motor (part of custom library) initial declaration
    //public final DcMotorEx lazySusan; //Special Intake Motor (part of custom library) initial declaration
    public final Servo gripper; //Bucket Mechanism Servo initial declaration

    //"Constructor" object for Delivery
    /*public Delivery(DcMotorEx slide, DcMotorEx lazySusan, Servo gripper, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer);
        this.slide=slide;
        this.lazySusan=lazySusan;
        this.gripper=gripper;
    }*/

    //"Constructor" object for Delivery
    public Delivery(DcMotorEx slide, Servo gripper, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer);
        this.slide=slide;
        this.gripper=gripper;
    }

    public void getEncoderValues(){
        telemetry.addData("Slide Encoder Position: ", slide.getCurrentPosition());
        //telemetry.addData("LazySusan Encoder Position: ", lazySusan.getCurrentPosition());
        telemetry.update();
    }

    public void initEncoders(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void stopSlide(){
        slide.setPower(0);
    }

    public int slideIdxToEncoderVal(int idx){

        //Probably redundant, buuut better safe than sorry
        if(idx<0){
            idx=0;
        }
        else if(idx>3){
            idx=3;
        }


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

    public void slideHigh(){
        slide.setTargetPosition(4250);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.65);
    }

    public void slidePickupStack(){
        slide.setTargetPosition(600);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.65);
    }

    public void resetEncoders(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*public void runLazySusan(int pos, double power){
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setTargetPosition(pos);
        lazySusan.setPower(power);
        lazySusan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopLazySusan(){
        lazySusan.setPower(0);
    }*/

    public void runGripper(double pos){
        gripper.setPosition(pos);
    }

    public void openGripper(){
        gripper.setPosition(0.225);
    }


    public void closeGripper(){
        gripper.setPosition(0);
    }


}
