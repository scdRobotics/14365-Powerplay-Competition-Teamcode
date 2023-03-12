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


    int SKEW_COUNT = 0;



        /*
        BASIC MOVEMENTS (UNIVERSAL)
        */


    int START_X = 36;
    double START_Y = 61.25;
    double START_ANG = 270;
    public int I_APPROACH_TURN = 45;

    int I_APPROACH_X = 36;
    int I_APPROACH_Y = 12;

    public int I_TOWARDS_POLE = 6;


    double I_DROP_X = 32.19;
    double I_DROP_Y = 6.19;


    int I_BACK_POLE_X = 36;
    int I_BACK_POLE_Y = 12;
    int I_BACK_POLE_ANG = 360;


    double I_PKUP_X = 67;
    int I_PKUP_Y = 12;


    double I_PKUP_BKUP_X = 66;
    int I_PKUP_BKUP_Y = 12;


    int II_APPROACH_X = 36;
    int II_APPROACH_Y = 12;
    int II_APPROACH_TURN = 135;


    double II_DROP_X = 30.19;
    double II_DROP_Y = 6.19;



    double POLE_WAIT_DROP = 0.75;
    double POLE_WAIT_RELEASE = 0.25;
    double STACK_WAIT_GRAB = 0.5;
    double STACK_WAIT_UP = 1.25;

    int II_BACK_POLE_X = 36;
    int II_BACK_POLE_Y = 12;
    int II_BACK_POLE_ANG = 360;


    double II_PKUP_X = 67.5;
    int II_PKUP_Y = 12;


    double II_PKUP_BKUP_X = 60.75;
    int II_PKUP_BKUP_Y = 12;



    double III_DROP_X = 30.19;
    double III_DROP_Y = 6.19;

    int III_APPROACH_X = 36;
    int III_APPROACH_Y = 12;
    int III_APPROACH_TURN = 135;



    int III_BACK_POLE_X = 36;
    int III_BACK_POLE_Y = 12;
    int III_BACK_POLE_ANG = 360;





    int HIGH_POLE_DROP_HEIGHT = 4100;
    int MEDIUM_POLE_DROP_HEIGHT = 2700;
    int I_CONE_STACK_PICKUP_HEIGHT = 575;
    int II_CONE_STACK_PICKUP_HEIGHT = 475;

    double SLIDE_POWER = 0.7;



    int COLLISION_AVOIDANCE_LOWER_LIMIT = 4;
    int COLLISION_AVOIDANCE_UPPER_LIMIT = 12;





    boolean trajectorySkewFirst = false;

    boolean trajectorySkewSecond = false;

    boolean trajectorySkewThird = false;


    //NOTE: Only the distances have different tolerances for I and II/III dropoffs. This is because the first dropoff is at an angle and inherently different than the rest.
    int WEBCAM_DEGREE_TOLERANCE = 5; //Degree of tolerance within findDTheta to trigger a skew flag

    int I_WEBCAM_DIST_TOLERANCE = 3;

    int I_EXPECTED_WEBCAM_READOUT = 14;

    int II_III_WEBCAM_DIST_TOLERANCE = 3;

    int II_III_EXPECTED_WEBCAM_READOUT = 14;


    int I_DISTANCE_SENSOR_TOLERANCE = 1; //Degree of tolerance within front distance sensor readout to trigger a skew flag

    int I_EXPECTED_SENSOR_READOUT = 2; //TODO: NEEDS AN ACTUAL MEASUREMENT. Exepected readout of front distance sensor for first drop


    int II_III_DISTANCE_SENSOR_TOLERANCE = 1; //Degree of tolerance within front distance sensor readout to trigger a skew flag

    int II_III_EXPECTED_SENSOR_READOUT = 2; //TODO: NEEDS AN ACTUAL MEASUREMENT. Exepected readout of front distance sensor for first drop



    double PARK_II_X = 36+0.01+1.5;

    double LEFT_PARK_III_X = 12+1.5;
    int RIGHT_PARK_III_X = 60;

    double LEFT_PARK_I_X = 60+1.5;
    int RIGHT_PARK_I_X = 12;

    int PARK_Y = 12;

    int[] validRobotPosConversion = new int[6];




    double WEBCAM_THETA_ACCEPTABLE_RANGE = Math.toRadians(3);

    double WEBCAM_DIST_ACCEPTABLE_RANGE = 3;


    double ODO_THETA_ACCEPTABLE_RANGE = Math.toRadians(1);

    double ODO_DIST_ACCEPTABLE_RANGE = 1;

    double ODO_ACCEPTABLE_COMPARSION_RANGE = 1.5;

    double ODO_COORDS_EXPECTED = 12;



    double BLUE_LEFT_IDEAL_THETA = Math.toRadians(245);

    double FRONT_SENSOR_DIST_ACCEPTABLE_RANGE = 1.5;

    double IDEAL_DIST = 6.15;


    int IDEAL_POLE_X = 24;

    
    void initAuto(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, hardwareMap, telemetry, timer, false);

        robot.delivery.initEncoders();

        robot.vision.activateYellowPipelineCamera1();

        robot.vision.activateAprilTagYellowPipelineCamera2();

        robot.delivery.closeGripper();

        robot.sensors.deployOdo();

        robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

        //TODO: Test if this actually populates the array properly and its viewable in auto programs
        for(int i = 0; i< validRobotPosConversion.length; i++){
            validRobotPosConversion[i] = ((i*24) + 12) - 72;
        }

    }
    
    @Override
    public void runOpMode() throws InterruptedException{

    }
    
    

    
    
}
