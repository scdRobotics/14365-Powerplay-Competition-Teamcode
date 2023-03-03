package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
//Most imports are automatically handled by Android Studio as you program

/*
SPECIAL NOTE: OUTSIDE LIBRARY
Setup can be found at https://learnroadrunner.com/installing.html
 */
/*
SPECIAL NOTE: OUTSIDE LIBRARY
Setup can be found at https://github.com/OpenFTC/RevExtensions2
 */

public class Robot {

    //Subsytem Declaration
    public Delivery delivery;
    public Vision vision;
    public SampleMecanumDrive drive;
    public Sensors sensors;

    //THESE MOTORS ARE ONLY USED IN TELE-OP; Roadrunner takes care of Auto drive motors
    public DcMotorEx frontLeftM; //Front Left Drive Motor initial declaration
    public DcMotorEx frontRightM; //Front Right Drive Motor initial declaration
    public DcMotorEx backLeftM; //Back Left Drive Motor initial declaration
    public DcMotorEx backRightM; //Back Right Drive Motor initial declaration

    //Delivery Mechanisms
    public DcMotorEx slide;

    public Servo gripper;


    public Servo backOdo;
    public Servo rightOdo;
    public Servo leftOdo;


    public OpenCvCamera webcam1;

    public OpenCvCamera webcam2;


    public DistanceSensor front;



    //public BNO055IMU imu;







    public RevBlinkinLedDriver led;

    //NOTE: These are all basic, required aspects of the robot
    public final ElapsedTime timer;
    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    public BNO055IMU imu;

    //"Constructor" object for Robot-- only the most basic, necessary objects are included
    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, boolean isTeleOp) {
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
            drive = new SampleMecanumDrive(hardwareMap);
        }


        //Map Delivery System
        slide=hardwareMap.get(DcMotorEx.class,"slide");
        gripper=hardwareMap.get(Servo.class,"gripper");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Map Sensor System
        front = hardwareMap.get(DistanceSensor.class, "front");
        /*leftFront = hardwareMap.get(DistanceSensor.class, "leftFront");
        rightFront = hardwareMap.get(DistanceSensor.class, "rightFront");
        leftBack = hardwareMap.get(DistanceSensor.class, "leftBack");
        rightBack = hardwareMap.get(DistanceSensor.class, "rightBack");*/

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

        //Map Vision System
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        //FtcDashboard.getInstance().startCameraStream(webcam1, 0);
        //TODO: Needs testing, but will be very helpful!
        FtcDashboard.getInstance().startCameraStream(webcam2, 0);



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);



        backOdo = hardwareMap.get(Servo.class, "backOdo");
        leftOdo = hardwareMap.get(Servo.class, "leftOdo");
        rightOdo = hardwareMap.get(Servo.class, "rightOdo");



        /*imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);*/


        //delivery = new Delivery(slide, lazySusan, gripper, telemetry, hardwareMap, timer);
        delivery = new Delivery(slide, gripper, telemetry, hardwareMap, timer);

        vision = new Vision(webcam1, webcam2, telemetry, hardwareMap, timer);

        sensors = new Sensors(imu, front, led, backOdo, leftOdo, rightOdo, telemetry, hardwareMap, timer);





    }

    //Quick telemetry function. Helps avoid conflicts caused by having telemetry in several threads
    public void telemetry(String caption, String value){
        telemetry.addData(caption, value);
        telemetry.update();
    }

    //Simple pause function-- basic, necessary function for all aspects of robot
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs && !opMode.isStopRequested()){

        }
    }
}
