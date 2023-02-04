package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    public double START_Y = 61.25;
    public double START_ANG = 180;


    public int I_APPROACH_X = 36;
    public int I_APPROACH_Y = 0;


    public int I_DROP_X = 32;
    public int I_DROP_Y = 0;


    public int I_BACK_POLE_X = 36;
    public int I_BACK_POLE_Y = 12;
    public int I_BACK_POLE_ANG = 360;


    public double I_PKUP_X = 61.25;
    public int I_PKUP_Y = 12;


    public double I_PKUP_BKUP_X = 60.75;
    public int I_PKUP_BKUP_Y = 12;


    public int II_APPROACH_X = 36;
    public int II_APPROACH_Y = 12;
    public int II_APPROACH_TURN = 135;


    public int II_DROP_X = 30;
    public int II_DROP_Y = 6;



    public int POLE_WAIT_DROP = 1;
    public double POLE_WAIT_RELEASE = 0.5;
    public int STACK_WAIT_GRAB = 1;
    public double STACK_WAIT_UP = 0.5;

    public int II_BACK_POLE_X = 38;
    public int II_BACK_POLE_Y = 12;
    public int II_BACK_POLE_ANG = 360;


    public double II_PKUP_X = 61.25;
    public int II_PKUP_Y = 12;


    public double II_PKUP_BKUP_X = 60.75;
    public int II_PKUP_BKUP_Y = 12;



    public int III_DROP_X = 30;
    public int III_DROP_Y = 6;

    public int III_APPROACH_X = 36;
    public int III_APPROACH_Y = 12;
    public int III_APPROACH_TURN = 135;



    public int III_BACK_POLE_X = 36;
    public int III_BACK_POLE_Y = 12;
    public int III_BACK_POLE_ANG = 360;





    public int HIGH_POLE_DROP_HEIGHT = 3900;
    public int MEDIUM_POLE_DROP_HEIGHT = 2700;
    public int I_CONE_STACK_PICKUP_HEIGHT = 500;
    public int II_CONE_STACK_PICKUP_HEIGHT = 435;

    public double SLIDE_POWER = 0.65;



    public int COLLISION_AVOIDANCE_LOWER_LIMIT = 4;
    public int COLLISION_AVOIDANCE_UPPER_LIMIT = 12;

    public double PARK_II_X = 36+0.01;

    public int LEFT_PARK_III_X = 12;
    public int RIGHT_PARK_III_X = 60;

    public int LEFT_PARK_I_X = 60;
    public int RIGHT_PARK_I_X = 12;

    public int PARK_Y = 12;

    
    
}
