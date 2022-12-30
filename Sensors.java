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

    private DistanceSensor left; //Left Dist Sensor initial declaration

    private DistanceSensor right; //Front Left Dist Sensor initial declaration

    //"Constructor" object for Sensors
    public Sensors(DistanceSensor front, DistanceSensor left, DistanceSensor right, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer); //Map basic, required aspects of robot

        this.front=front;
        this.left=left;
        this.right=right;
    }

    //NOTE: All below functions focus on updating particular distance sensor values. Pretty straightforward.

    public double getFrontDist(){
        return front.getDistance(DistanceUnit.INCH);
    }

    public double getLeftDist(){
        return left.getDistance(DistanceUnit.INCH);
    }

    public double getRightDist(){
        return right.getDistance(DistanceUnit.INCH);
    }

}