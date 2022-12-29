package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//Most imports are automatically handled by Android Studio as you program

public abstract class Subsystem {

    //NOTE: These are all basic, required aspects of the robot
    protected final Telemetry telemetry;
    protected final ElapsedTime timer;
    protected final HardwareMap hardwareMap;

    //"Constructor" object for Subsystem-- only the most basic, necessary objects are included
    public Subsystem(Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        this.telemetry=telemetry;
        this.hardwareMap=hardwareMap;
        this.timer=timer;

    }



}
