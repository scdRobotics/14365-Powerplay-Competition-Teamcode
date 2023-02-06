package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Most imports are automatically handled by Android Studio as you program

public class Sensors extends Subsystem {
    private DistanceSensor front; //Underbelly Dist Sensor #1 initial declaration

    private DistanceSensor leftFront; //Left Dist Sensor initial declaration

    private DistanceSensor rightFront; //Front Left Dist Sensor initial declaration

    private DistanceSensor leftBack; //Left Dist Sensor initial declaration

    private DistanceSensor rightBack; //Front Left Dist Sensor initial declaration

    //"Constructor" object for Sensors
    public Sensors(DistanceSensor front, DistanceSensor leftFront, DistanceSensor rightFront, DistanceSensor leftBack, DistanceSensor rightBack, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer); //Map basic, required aspects of robot

        this.front=front;
        this.leftFront=leftFront;
        this.rightFront=rightFront;
        this.leftBack=leftBack;
        this.rightBack=rightBack;
    }

    //NOTE: All below functions focus on updating particular distance sensor values. Pretty straightforward.

    public double getFrontDist(){
        return front.getDistance(DistanceUnit.INCH);
    }

    public double getLeftFrontDist(){
        return leftFront.getDistance(DistanceUnit.INCH);
    }

    public double getRightFrontDist(){
        return rightFront.getDistance(DistanceUnit.INCH);
    }

    public double getLeftBackDist(){
        return leftBack.getDistance(DistanceUnit.INCH);
    }

    public double getRightBackDist(){
        return rightBack.getDistance(DistanceUnit.INCH);
    }

}