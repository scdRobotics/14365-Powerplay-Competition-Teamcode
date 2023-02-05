package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTO_PRIME", group = "Autonomous")
@Disabled
public class AUTO_PRIME extends LinearOpMode {

    Robot robot;

    int closestX = 0;
    double closestTempValX = 100;
    int closestY = 0;
    double closestTempValY = 100;
    int closestAngle = 0;
    double closestTempValAngle = 500;



        /*
        BASIC MOVEMENTS (UNIVERSAL)
        */

    int START_X = 36;
    double START_Y = 61.25;
    double START_ANG = 180;


    int I_APPROACH_X = 36;
    int I_APPROACH_Y = 0;


    int I_DROP_X = 32;
    int I_DROP_Y = 0;


    int I_BACK_POLE_X = 36;
    int I_BACK_POLE_Y = 12;
    int I_BACK_POLE_ANG = 360;


    double I_PKUP_X = 61.25;
    int I_PKUP_Y = 12;


    double I_PKUP_BKUP_X = 60.75;
    int I_PKUP_BKUP_Y = 12;


    int II_APPROACH_X = 36;
    int II_APPROACH_Y = 12;
    int II_APPROACH_TURN = 135;


    int II_DROP_X = 30;
    int II_DROP_Y = 6;



    int POLE_WAIT_DROP = 1;
    double POLE_WAIT_RELEASE = 0.5;
    int STACK_WAIT_GRAB = 1;
    double STACK_WAIT_UP = 0.5;

    int II_BACK_POLE_X = 38;
    int II_BACK_POLE_Y = 12;
    int II_BACK_POLE_ANG = 360;


    double II_PKUP_X = 61.25;
    int II_PKUP_Y = 12;


    double II_PKUP_BKUP_X = 60.75;
    int II_PKUP_BKUP_Y = 12;



    int III_DROP_X = 30;
    int III_DROP_Y = 6;

    int III_APPROACH_X = 36;
    int III_APPROACH_Y = 12;
    int III_APPROACH_TURN = 135;



    int III_BACK_POLE_X = 36;
    int III_BACK_POLE_Y = 12;
    int III_BACK_POLE_ANG = 360;





    int HIGH_POLE_DROP_HEIGHT = 3900;
    int MEDIUM_POLE_DROP_HEIGHT = 2700;
    int I_CONE_STACK_PICKUP_HEIGHT = 500;
    int II_CONE_STACK_PICKUP_HEIGHT = 435;

    double SLIDE_POWER = 0.65;



    int COLLISION_AVOIDANCE_LOWER_LIMIT = 4;
    int COLLISION_AVOIDANCE_UPPER_LIMIT = 12;

    double PARK_II_X = 36+0.01;

    int LEFT_PARK_III_X = 12;
    int RIGHT_PARK_III_X = 60;

    int LEFT_PARK_I_X = 60;
    int RIGHT_PARK_I_X = 12;

    int PARK_Y = 12;

    int[] validRobotPosConversion = new int[6];

    
    void initAuto(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, hardwareMap, telemetry, timer, false);

        robot.delivery.initEncoders();

        robot.vision.activateYellowPipelineCamera1();

        robot.vision.activateAprilTagYellowPipelineCamera2();

        robot.delivery.closeGripper();

        //TODO: Test if this actually populates the array properly and its viewable in auto programs
        for(int i = 0; i< validRobotPosConversion.length; i++){
            validRobotPosConversion[i] = ((i*24) + 12) - 72;
        }

    }
    
    @Override
    public void runOpMode() throws InterruptedException{

    }
    
    

    
    
}