package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

    private RevBlinkinLedDriver led;

    public enum LED_STATE{
        DEFAULT,
        DESYNCED,
        POLE_GOOD,
        POLE_BAD,
        SEMI_AUTO
    }

    LED_STATE current = LED_STATE.DEFAULT;

    boolean isBlue = PoseTransfer.isBlue;

    //"Constructor" object for Sensors
    public Sensors(DistanceSensor front, DistanceSensor left, DistanceSensor right, RevBlinkinLedDriver led, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer); //Map basic, required aspects of robot

        this.front=front;
        this.left=left;
        this.right=right;

        this.led = led;

        updateLEDs();
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

    public void setLEDs(RevBlinkinLedDriver.BlinkinPattern b){
        led.setPattern(b);
    }

    public void setLEDState(LED_STATE c){
        if(current == c){
            return;
        }
        current = c;
        updateLEDs();

    }

    public void updateLEDs(){
        switch(current){

            case DEFAULT:
                if(isBlue){
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                }
                else{
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                break;

            case DESYNCED:
                if(isBlue){
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                }
                else{
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                }
                break;

            case POLE_BAD:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;

            case POLE_GOOD:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;

            case SEMI_AUTO:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;

        }
    }

}