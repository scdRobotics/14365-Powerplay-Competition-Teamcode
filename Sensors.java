package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Most imports are automatically handled by Android Studio as you program

public class Sensors extends Subsystem {
    private DistanceSensor groundFront; //Underbelly Dist Sensor #1 initial declaration
    public double groundFrontDist; //Create blank value for underbelly dist sensor #1 readouts
    private DistanceSensor groundBack; //Underbelly Dist Sensor #2 initial declaration
    public double groundBackDist; //Create blank value for underbelly dist sensor #2 readouts

    private DistanceSensor left; //Left Dist Sensor initial declaration
    public double leftDist; //Create blank value for left dist sensor
    private DistanceSensor right; //Right Dist Sensor initial declaration
    public double rightDist; //Create blank value for right dist sensor

    private DistanceSensor frontLeft; //Front Left Dist Sensor initial declaration
    public double frontLeftDist; //Create blank value for front left dist sensor
    private DistanceSensor frontRight; //Front Right Dist Sensor initial declaration
    public double frontRightDist; //Create blank value for front right dist sensor

    private DistanceSensor backLeft; //Back Left Dist Sensor initial declaration
    public double backLeftDist; //Create blank value for back left dist sensor
    private DistanceSensor backRight; //Back Right Dist Sensor initial declaration
    public double backRightDist; //Create blank value for back right dist sensor

    //"Constructor" object for Sensors
    public Sensors(DistanceSensor groundFront, DistanceSensor groundBack, DistanceSensor left, DistanceSensor right, DistanceSensor frontLeft, DistanceSensor frontRight, DistanceSensor backLeft, DistanceSensor backRight, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer); //Map basic, required aspects of robot

        this.groundFront=groundFront; //Underbelly dist sensor #1 map
        this.groundBack=groundBack; //Underbelly dist sensor #2 map
        this.left=left; //Left dist sensor map
        this.right=right; //Right dist sensor map
        this.frontRight=frontRight; //Front right dist sensor map
        this.frontLeft=frontLeft; //Front left dist sensor map
        this.backLeft=backLeft; //Back left dist sensor map
        this.backRight=backRight; //Back right dist sensor map
    }

    //NOTE: All below functions focus on updating particular distance sensor values. Pretty straightforward.

    public void updateAllDist(){
        groundFrontDist=groundFront.getDistance(DistanceUnit.CM);
        groundBackDist=groundFront.getDistance(DistanceUnit.CM);

        leftDist=left.getDistance(DistanceUnit.CM);
        rightDist=right.getDistance(DistanceUnit.CM);

        frontLeftDist=frontLeft.getDistance(DistanceUnit.CM);
        frontRightDist=frontRight.getDistance(DistanceUnit.CM);

        backLeftDist=backLeft.getDistance(DistanceUnit.CM);
        backRightDist=backRight.getDistance(DistanceUnit.CM);
    }

    public void updateSideDist(){
        leftDist=left.getDistance(DistanceUnit.CM);
        rightDist=right.getDistance(DistanceUnit.CM);

        frontLeftDist=frontLeft.getDistance(DistanceUnit.CM);
        frontRightDist=frontRight.getDistance(DistanceUnit.CM);

        backLeftDist=backLeft.getDistance(DistanceUnit.CM);
        backRightDist=backRight.getDistance(DistanceUnit.CM);
    }

    public void updateGroundDist(){
        groundFrontDist=groundFront.getDistance(DistanceUnit.CM);
        groundBackDist=groundFront.getDistance(DistanceUnit.CM);
    }

    public void updateRightDist(){
        rightDist=right.getDistance(DistanceUnit.CM);
    }

    public void updateLeftDist(){
        leftDist=left.getDistance(DistanceUnit.CM);
    }

    public void updateFrontDist(){
        frontLeftDist=frontLeft.getDistance(DistanceUnit.CM);
        frontRightDist=frontRight.getDistance(DistanceUnit.CM);
    }

    public void updateBackDist(){
        backLeftDist=backLeft.getDistance(DistanceUnit.CM);
        backRightDist=backRight.getDistance(DistanceUnit.CM);
    }

    public void updateFrontLeftDist(){
        frontLeftDist=frontLeft.getDistance(DistanceUnit.CM);
    }

    public void updateFrontRightDist(){
        frontRightDist=frontRight.getDistance(DistanceUnit.CM);
    }

    public void updateBackLeftDist(){
        backLeftDist=backLeft.getDistance(DistanceUnit.CM);
    }

    public void updateBackRightDist(){
        backRightDist=backRight.getDistance(DistanceUnit.CM);
    }

}