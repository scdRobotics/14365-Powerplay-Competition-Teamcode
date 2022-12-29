package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//Most imports are automatically handled by Android Studio as you program

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
/*
SPECIAL NOTE: OUTSIDE LIBRARY
Setup can be found at https://learnroadrunner.com/installing.html
 */

/*
SPECIAL NOTE: OUTSIDE LIBRARY
Setup can be found at https://github.com/OpenFTC/RevExtensions2
 */

public class RobotDrive {

    // THESE MOTORS ARE ONLY USED IN TELE-OP; Roadrunner takes care of Auto drive motors
    public DcMotorEx frontLeftM; //Front Left Drive Motor initial declaration
    public DcMotorEx frontRightM; //Front Right Drive Motor initial declaration
    public DcMotorEx backLeftM; //Back Left Drive Motor initial declaration
    public DcMotorEx backRightM; //Back Right Drive Motor initial declaration

    //NOTE: These are all basic, required aspects of the robot
    public final ElapsedTime timer;
    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    //"Constructor" object for Robot-- only the most basic, necessary objects are included
    public RobotDrive(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, boolean isTeleOp) {
        this.opMode = opMode;
        this.timer = timer;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.init(isTeleOp);
    }

    /*
    NOTE: Initialization for Robot object. Works by:
        1. Mapping previously declared mechanisms
        2. Declaring subsytem objects with mapped mechanisms
     */
    public void init(boolean isTeleOp){

        //NOTE: Only need to map motors in teleop, otherwise Roadrunner takes care of motor mappings
        if(isTeleOp){
            frontRightM=hardwareMap.get(DcMotorEx.class,"frontRight");
            frontLeftM=hardwareMap.get(DcMotorEx.class,"frontLeft");
            backLeftM=hardwareMap.get(DcMotorEx.class,"backLeft");
            backRightM=hardwareMap.get(DcMotorEx.class,"backRight");

            frontLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftM.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of front left motor to use power, NOT encoders
            frontRightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of front right motor to use power, NOT encoders
            backLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of back left motor to use power, NOT encoders
            backRightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of back right motor to use power, NOT encoders

            frontLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        else {
            //Initialize Autonomous Drive System
        }



    }

    //Quick telemetry function. Helps avoid conflicts caused by having telemetry in several threads
    public void telemetry(String caption, String value){
        telemetry.addData(caption, value);
        telemetry.update();
    }

    //Simple pause function-- basic, necessary function for all aspects of robot
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs){

        }
    }
}
