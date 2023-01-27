package org.firstinspires.ftc.teamcode;

//TODO: Integrate into other auto programs, not just _BLUE_LEFT_AUTO

public class AUTO_CONSTANTS {

    /*
    BASIC MOVEMENTS (UNIVERSAL)
     */

    public static final double START_X = 36;
    public static final double START_Y = 63.5;

    public static final double FIRST_APPROACH_X = 36;
    public static final double FIRST_APPROACH_Y = 4;

    public static final double FIRST_ALIGN_POLE_X = 36;
    public static final double FIRST_ALIGN_POLE_Y = 12;

    public static final double FIRST_BACK_OFF_FROM_POLE_X = 34;
    public static final double FIRST_BACK_OFF_FROM_POLE_Y = 12;

    public static final double CONE_STACK_X = 63.5 + 0.15;
    public static final double CONE_STACK_Y = 12;

    public static final double CONE_STACK_X_BACKUP = 62.75 + 0.15;
    public static final double CONE_STACK_Y_BACKUP = 12;

    public static final double SECOND_ALIGN_POLE_X = 36;
    public static final double SECOND_ALIGN_POLE_Y = 12;

    public static final double SECOND_BACK_OFF_FROM_POLE_X = 30;
    public static final double SECOND_BACK_OFF_FROM_POLE_Y = 12;

    public static final double PARK_1_X_LEFT = 60;
    public static final double PARK_1_X_RIGHT = 12;
    public static final double PARK_1_Y = 12;

    public static final double PARK_2_X = 36;
    public static final double PARK_2_Y = 12;

    public static final double PARK_3_X_LEFT = 12;
    public static final double PARK_3_X_RIGHT = 60;
    public static final double PARK_3_Y = 12;


    public static final double START_ANGLE = 270;

    /*
    TODO: MAY CAUSE ISSUES WITH DIFFERENT VERSIONS
     */

    public static final double ALIGN_POLE_ANGLE = 315 - 6.5; //BLUE RIGHT ONLY
    //BLUE LEFT = 315 - 90 = 225
    //RED RIGHT = 315 - 180 = 135
    //RED RIGHT = 315 - 270 = 45

    public static final double CONE_STACK_TURN_TOWARD_ANGLE = 135 + 6.5;


    /*
    ALTERNATE MOVEMENTS (UNIVERSAL)
     */

    public static final double ALT_START_X = 36;
    public static final double ALT_START_Y = 36;
    public static final double ALT_START_ANGLE = 270;

    public static final double ALT_BACK_OFF_FROM_POLE_X = 34;
    public static final double ALT_BACK_OFF_FROM_POLE_Y = 36;


    public static final double ALT_ALIGN_POLE_ANGLE = 45;

    public static final double ALT_CONE_STACK_TURN_TOWARD_ANGLE = 135;

    public static final double ALT_PARK_1_X_LEFT = 60;
    public static final double ALT_PARK_1_X_RIGHT = 12;
    public static final double ALT_PARK_1_Y = 36;

    public static final double ALT_PARK_2_X = 36;
    public static final double ALT_PARK_2_Y = 36;

    public static final double ALT_PARK_3_X_LEFT = 12;
    public static final double ALT_PARK_3_X_RIGHT = 60;
    public static final double ALT_PARK_3_Y = 36;



    /*
    SENSOR VALUES (UNIVERSAL)
     */

    public static final double FIRST_ROBOT_DISTANCE_UPPER_LIMIT = 10;
    public static final double FIRST_ROBOT_DISTANCE_LOWER_LIMIT = 5;

    public static final double FIRST_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 3;
    public static final double FIRST_POLE_DISTANCE_UPPER_LIMIT = 12;
    public static final double FIRST_POLE_DEFAULT_TRAVEL_DIST = 6.7;

    public static final double SECOND_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 2.5;
    public static final double SECOND_POLE_DISTANCE_UPPER_LIMIT = 12;
    public static final double SECOND_POLE_DEFAULT_TRAVEL_DIST = 8.5;


    public static final double ALT_POLE_DISTANCE_SUBTRACTIVE_MODIFIER = 0.5;
    public static final double ALT_POLE_DISTANCE_UPPER_LIMIT = 12;
    public static final double ALT_POLE_DEFAULT_TRAVEL_DIST = 6;


    public static final double POLE_WAIT_DROP = 0.75;
    public static final double POLE_WAIT_RELEASE = 0.25;




}
