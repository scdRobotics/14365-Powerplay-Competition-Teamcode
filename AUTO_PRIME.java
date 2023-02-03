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

    public int FIRST_APPROACH_X = 36;
    public int FIRST_APPROACH_Y = 4;

    public int FIRST_ALIGN_POLE_X = 36;
    public int FIRST_ALIGN_POLE_Y = 12;

    public int FIRST_BACK_OFF_FROM_POLE_X = 34;
    public int FIRST_BACK_OFF_FROM_POLE_Y = 12;

    public double CONE_STACK_X = 63.5 + 0.15;
    public int CONE_STACK_Y = 12;

    public double CONE_STACK_X_BACKUP = 62.75 + 0.15;
    public int CONE_STACK_Y_BACKUP = 12;

    public int SECOND_ALIGN_POLE_X = 36;
    public int SECOND_ALIGN_POLE_Y = 12;

    public int SECOND_BACK_OFF_FROM_POLE_X = 30;
    public int SECOND_BACK_OFF_FROM_POLE_Y = 12;

    public int PARK_1_X_LEFT = 60;
    public int PARK_1_X_RIGHT = 12;
    public int PARK_1_Y = 12;

    public int PARK_2_X = 36;
    public int PARK_2_Y = 12;

    public int PARK_3_X_LEFT = 12;
    public int PARK_3_X_RIGHT = 60;
    public int PARK_3_Y = 12;


    public int START_ANGLE = 270;

    /*
    TODO: MAY CAUSE ISSUES WITH DIFFERENT VERSIONS (UPDATE: IT DOES. PLS FIX LATER.)
     */

    public double ALIGN_POLE_ANGLE = 315 - 6.5; //BLUE RIGHT ONLY
    //BLUE LEFT = 315 - 90 = 225
    //RED RIGHT = 315 - 180 = 135
    //RED RIGHT = 315 - 270 = 45

    public double CONE_STACK_TURN_TOWARD_ANGLE = 135 + 6.5;


    /*
    ALTERNATE MOVEMENTS (UNIVERSAL)
     */

    public int ALT_START_X = 36;
    public int ALT_START_Y = 36;
    public int ALT_START_ANGLE = 270;

    public int ALT_BACK_OFF_FROM_POLE_X = 34;
    public int ALT_BACK_OFF_FROM_POLE_Y = 36;


    public int ALT_ALIGN_POLE_ANGLE = 45;

    public int ALT_CONE_STACK_TURN_TOWARD_ANGLE = 135;

    public int ALT_PARK_1_X_LEFT = 60;
    public int ALT_PARK_1_X_RIGHT = 12;
    public int ALT_PARK_1_Y = 36;

    public int ALT_PARK_2_X = 36;
    public int ALT_PARK_2_Y = 36;

    public int ALT_PARK_3_X_LEFT = 12;
    public int ALT_PARK_3_X_RIGHT = 60;
    public int ALT_PARK_3_Y = 36;



    /*
    SENSOR VALUES (UNIVERSAL)
     */

    public int FIRST_ROBOT_DISTANCE_UPPER_LIMIT = 10;
    public int FIRST_ROBOT_DISTANCE_LOWER_LIMIT = 5;

    public int FIRST_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 3;
    public int FIRST_POLE_DISTANCE_UPPER_LIMIT = 12;
    public double FIRST_POLE_DEFAULT_TRAVEL_DIST = 6.7;

    public double SECOND_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 2.5;
    public double SECOND_POLE_DISTANCE_UPPER_LIMIT = 12;
    public double SECOND_POLE_DEFAULT_TRAVEL_DIST = 8.5;


    public double ALT_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 0.5;
    public double ALT_POLE_DISTANCE_UPPER_LIMIT = 12;
    public double ALT_POLE_DEFAULT_TRAVEL_DIST = 6;


    public double POLE_WAIT_DROP = 0.75;
    public double POLE_WAIT_RELEASE = 0.25;


    public int HIGH_POLE_DROP_HEIGHT = 3900;
    public int MEDIUM_POLE_DROP_HEIGHT = 2700;
    public int CONE_STACK_PICKUP_HEIGHT = 500;

    public double SLIDE_POWER = 0.65;

    
    
    
}
