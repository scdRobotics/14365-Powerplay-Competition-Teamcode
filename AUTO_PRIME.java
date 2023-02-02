package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTO_PRIME", group = "Autonomous")
@Disabled
public class AUTO_PRIME extends LinearOpMode {

    public Robot robot;
    
    public void initAuto(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, hardwareMap, telemetry, timer, false);

        robot.delivery.initEncoders();

        robot.vision.activateYellowPipelineCamera1();

        robot.vision.activateAprilTagYellowPipelineCamera2();

        robot.delivery.closeGripper();
    }
    
    @Override
    public void runOpMode() throws InterruptedException{

    }
    
    /*
    BASIC MOVEMENTS (UNIVERSAL)
     */

    public int START_X = 36;
    public double START_Y = 63.5;
    public double START_ANGLE = 180;

    public int FIRST_APPROACH_X = 36;
    public int FIRST_APPROACH_Y = 0;

    public int FIRST_DROP_X = 32;
    public int FIRST_DROP_Y = 0;

    public int FIRST_BACKOFF_POLE_X = 36;
    public int FIRST_BACKOFF_POLE_Y = 12;
    public int FIRST_BACKOFF_POLE_ANGLE = 360;


    public double POLE_WAIT_DROP = 1;
    public double POLE_WAIT_RELEASE = 0.5;


    public int HIGH_POLE_DROP_HEIGHT = 3900;
    public int MEDIUM_POLE_DROP_HEIGHT = 2700;
    public int CONE_STACK_PICKUP_HEIGHT = 500;

    public double SLIDE_POWER = 0.65;



    public int COLLISION_AVOIDANCE_LOWER_LIMIT = 4;
    public int COLLISION_AVOIDANCE_UPPER_LIMIT = 12;

    
    
    
}
