package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//Most imports are automatically handled by Android Studio as you program

@Autonomous(name = "AutoPrime", group = "Concept")
@Disabled
public class AutoPrime extends LinearOpMode {
    public RobotDrive robot; //Basic robot object

    public static boolean armDump = false; //Autonomous Only Variable

    public static boolean intakeSpin = false; //Autonomous Only Variable

    //Initialize everything specific to autonomous
    public void initAuto(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new RobotDrive(this, hardwareMap, telemetry, timer, false);
    }

    //Necessary for crash handling
    @Override
    public void runOpMode() throws InterruptedException {
    }
}
