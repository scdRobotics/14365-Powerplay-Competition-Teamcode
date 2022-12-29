package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualDrive extends Subsystem {

    public final DcMotorEx frontLeft; //Special Intake Motor (part of custom library) initial declaration
    public final DcMotorEx frontRight; //Special Intake Motor (part of custom library) initial declaration
    public final DcMotorEx backLeft; //Special Intake Motor (part of custom library) initial declaration
    public final DcMotorEx backRight; //Special Intake Motor (part of custom library) initial declaration

    //"Constructor" object for Delivery
    public ManualDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry,hardwareMap,timer);
        this.frontLeft=frontLeft;
        this.frontRight=frontRight;
        this.backLeft=backLeft;
        this.backRight=backRight;
    }



}
