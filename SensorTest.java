package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="SensorTest", group="TestAutonomous")
public class SensorTest extends AUTO_PRIME {

    @Override
    public void runOpMode() {


        ElapsedTime timer = new ElapsedTime();

        initAuto();

        robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

        waitForStart();

        robot.delivery.slideControl(HIGH_POLE_DROP_HEIGHT, SLIDE_POWER);

        robot.pause(2);

        /*
        BLUE LEFT ESTIMATES
         */


        //robot.drive.setPoseEstimate(new Pose2d(36, 12, Math.toRadians(225)));

        /*isSkew(robot.drive.getPoseEstimate().getHeading(),
                robot.vision.findClosePoleDTheta(),
                robot.sensors.getFrontDist(),
                (robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/Math.sin(robot.drive.getPoseEstimate().getHeading() - Math.toRadians(180)),
                robot.vision.findClosePoleDist());*/



        ArrayList<Double> allDTheta = new ArrayList<Double>();
        ArrayList<Double> allDist = new ArrayList<Double>();

        //telemetry.addData("Camera DTheta (Before Turn): ", dTheta);
        //telemetry.addData("Camera Dist (Before Movement): ", dist);
        //telemetry.update();

        robot.pause(5);

        robot.timer.startTime();

        int i = 0;
        while(!isStopRequested() && i<100){
            i++;
            telemetry.addData("Passthrough # ", + i);
            telemetry.update();
            allDTheta.add(robot.vision.findClosePoleDTheta());
            allDist.add(robot.vision.findClosePoleDist());
            robot.pause(0.15);
        }

        double meanDTheta = 0;
        double meanDist = 0;

        double dThetaSum = 0;
        double distSum = 0;

        int validCountDTheta = 0;
        for(Double d: allDTheta){
            if(d!=-1){
                dThetaSum += d;
                validCountDTheta++;
            }
        }

        int validCountDist = 0;
        for(Double d: allDist){
            if(d!=-1){
                distSum += d;
                validCountDist++;
            }

        }

        meanDTheta = dThetaSum/validCountDTheta;

        meanDist = distSum/validCountDist;

        double standDevDThetaSum = 0;
        double standDevDistSum = 0;

        for(Double d: allDTheta){
            if(d!=-1){
                standDevDThetaSum += Math.pow(meanDTheta-d, 2);
            }
        }

        standDevDThetaSum = Math.sqrt(standDevDThetaSum/validCountDTheta);


        for(Double d: allDist){
            if(d!=-1){
                standDevDistSum += Math.pow(meanDist-d, 2);
            }
        }

        standDevDistSum = Math.sqrt(standDevDistSum/validCountDist);

        telemetry.addData("Mean dTheta: ", meanDTheta);
        telemetry.addData("Mean dist: ", meanDist);

        telemetry.addData("dTheta Stand Dev: ", standDevDThetaSum);
        telemetry.addData("dist Stand Dev: ", standDevDistSum);

        telemetry.addData("Valid dTheta %: ", validCountDTheta);
        telemetry.addData("Valid dist %", validCountDist);

        int count = 0;
        for(Double dTheta: allDTheta){
            count++;
            telemetry.addData("dTheta #" + count + " Val: ", dTheta/100);
        }

        count = 0;
        for(Double dist: allDist){
            count++;
            telemetry.addData("dist #" + count + " Val: ", dist/100);
        }

        telemetry.update();

        robot.pause(100);






    }


    public boolean isSkew(double localizerTheta, double camTheta, double frontSensorDist, double localizerDist, double camDist){

        /*
        TELEMETRY CALLS
         */

        telemetry.addData("-----Skew Passthrough #-----", SKEW_COUNT);

        telemetry.addData("Localizer Theta: ", localizerTheta);
        telemetry.addData("Cam Theta: ", camTheta);
        telemetry.addData("Front Sensor Dist: ", frontSensorDist);
        telemetry.addData("Localizer Dist: ", localizerDist);
        telemetry.addData("Cam Dist: ", camDist);

        telemetry.addData("Is Localizer Theta Accurate? ", isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA));
        telemetry.addData("Is Camera Theta Accurate? ", isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA));
        telemetry.addData("Is Front Sensor Dist Accurate? ", isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));
        telemetry.addData("Is Localizer Dist Accurate? ", isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));
        telemetry.addData("Is Webcam Dist Accurate? ", isEqual(camDist, WEBCAM_DIST_ACCEPTABLE_RANGE, IDEAL_DIST));

        telemetry.update();


        /*
        MAIN CHECK-- IS EVERYTHING GOOD?
         */
        if(
                isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)
                        && isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)
                        && isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
                        && isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
                        && isEqual(camDist, WEBCAM_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)
        ){
          /*
          IF EVERYTHING IS GOOD, NO SKEW! ELSE, CONTINUE
           */
            SKEW_COUNT=0;
            return false;
        }

        else{

            if(!isEqual(camTheta + BLUE_LEFT_IDEAL_THETA, Math.toRadians(WEBCAM_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)){ //IF CAM THETA IS NOT WHAT WE EXPECT...
                camThetaAccurate=false; //No longer flagged as accurate
                camTheta = robot.vision.findClosePoleDTheta(); //Update camera in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                camThetaAccurate=true; //Flagged as accurate
            }

            if(!isEqual(localizerTheta, Math.toRadians(ODO_THETA_ACCEPTABLE_RANGE), BLUE_LEFT_IDEAL_THETA)){ //IF LOCALIZER THETA IS NOT WHAT WE EXPECT...
                localizerThetaAccurate=false; //No longer flagged as accurate
                if(camThetaAccurate){ //If camera theta is accurate, we have a reference point! (Albeit, one with a good amount of noise we maybe shouldn't rely on... needs testing)
                    robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), camTheta + BLUE_LEFT_IDEAL_THETA)); //Update localizer estimate based on camTheta readout
                }
            }
            else{ //IF IT IS WHAT WE EXPECT...
                localizerThetaAccurate=true; //Flagged as accurate
            }

            if(!isEqual(camDist, Math.toRadians(WEBCAM_DIST_ACCEPTABLE_RANGE), IDEAL_DIST)){ //IF CAMERA DIST IS NOT WHAT WE EXPECT...
                camDistAccurate=false; //No longer flagged as accurate
                camDist = robot.vision.findClosePoleDist(); //Update camera in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                camDistAccurate=true; //Flagged as accurate
            }

            if(!isEqual(frontSensorDist, FRONT_SENSOR_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)){ //IF FRONT SENSOR DIST IS NOT WHAT WE EXPECT...
                frontSensorDistAccurate=false; //No longer flagged as accurate
                frontSensorDist = robot.sensors.getFrontDist(); //Update sensor in case it is a processing error
            }
            else{ //IF IT IS WHAT WE EXPECT...
                frontSensorDistAccurate=true; //Flagged as accurate
            }

            if(!isEqual(localizerDist, ODO_DIST_ACCEPTABLE_RANGE, IDEAL_DIST)){ //IF ODOMETRY DIST IS NOT WHAT WE EXPECT... (this one's a doozy)

                if(localizerThetaAccurate){ //IF LOCALIZER THETA (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                    double cos = Math.cos(localizerTheta - Math.toRadians(180)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/ cos, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated X is outside tolerance... (ask Logan about math- it's just trig triangles)

                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *frontSensorDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on sensor calculated X dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *camDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }


                    double sin = Math.sin(localizerTheta - Math.toRadians(180)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getY())/ sin, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated Y is outside tolerance... (ask Logan about math- it's just trig triangles)
                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *frontSensorDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer Y estimate based on sensor calculated Y dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *camDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }
                }


                else if(camThetaAccurate){ //IF CAMERA THETA (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                    double cos = Math.cos(camTheta + Math.toRadians(45)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getX()-IDEAL_POLE_X)/ cos, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated X is outside tolerance... (ask Logan about math- it's just trig triangles)

                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *frontSensorDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on sensor calculated X dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(cos *camDist, robot.drive.getPoseEstimate().getY(), robot.drive.getPoseEstimate().getX())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }


                    double sin = Math.sin(camTheta - Math.toRadians(45)); //Variable for storage
                    if(!isEqual((robot.drive.getPoseEstimate().getY())/ sin, ODO_ACCEPTABLE_COMPARSION_RANGE, ODO_COORDS_EXPECTED)){ //If our calculated Y is outside tolerance... (ask Logan about math- it's just trig triangles)
                        if(frontSensorDistAccurate){ //IF DISTANCE SENSOR (PREFERRED REFERENCE, LESS VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *frontSensorDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer Y estimate based on sensor calculated Y dist
                        }
                        else if(camDistAccurate){ //IF CAMERA DIST (NOT PREFERRED, MORE VARIANCE) IS ACCURATE...
                            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), sin *camDist, robot.drive.getPoseEstimate().getHeading())); //Update localizer X estimate based on camera calculated X dist
                        }
                    }
                }


                else{
                    //Can't do anything else :( too much is messed up and totally wrong. hope it fixes itself on a recursive call. this should never happen since we should always have at least the webcams viewing the pole but, you never know.
                }

                localizerDistAccurate=false; //No longer flagged as accurate

            }else{
                localizerDistAccurate=true; //Flagged as accurate
            }

            if(localizerDistAccurate && localizerThetaAccurate){
                //Just localizer is good enough to act on-- no skew here folks, juuust potentially faulty sensors!
                return false;
            }

            if(SKEW_COUNT<4){ //IF WE HAVENT RECURSIVE CALLED 4 OR MORE TIMES...
                SKEW_COUNT++; //Increase count
                isSkew(robot.drive.getPoseEstimate().getHeading(), camTheta, robot.drive.getPoseEstimate().getY()/Math.sin(camTheta - Math.toRadians(45)), localizerDist, camDist); //Recursive function yay! This just goes back and checks sensors again after aalll refreshes in logic above have occured...
            }

            robot.sensors.setLEDState(Sensors.LED_STATE.DESYNCED); //We did all recursive calls and still, something's wrong. Big uh oh. Just go to live built trajectories and hope robot can mostly fix itself, probably. NOTE: ITS EXTREMELY RARE FOR THIS TO HAPPEN SINCE IT MEANS EVERY SINGLE SENSOR HAS IN SOME WAY MESSED UP. FUNCTIONALLY, WE WILL NEVER REACH THIS-- BUT BETTER TO HOPE "IT'LL FIGURE IT OUT WITH LOCALIZER ADJUSTMENTS" THAN HAVE IT CRASH HERE
            return true; //We skewed and couldn't fix :(
        }

    }

    public boolean isEqual(double a, double aRange, double b, double bRange){
        return (a-aRange <= b+bRange) && (b-bRange <= a+aRange);
    }

    /*public boolean isEqual(double a, double aRange, double b){
        //return (a-aRange <= b) && (b <= a+aRange);
    }*/

    public boolean isEqual (double x, double delta, double a) //X = sensor input, A = ideal input, delta = range/2
    {
        return Math.abs(x-a) < (delta/2);
    }

}
