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

        robot.vision.runAprilTag(false);

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
            if(d!=-1 || d!=-0.01){
                dThetaSum += d;
                validCountDTheta++;
            }
        }

        int validCountDist = 0;
        for(Double d: allDist){
            if(d!=-1 || d!=-0.01){
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

        double medianDTheta = 0;
        double medianDist = 0;

        if (allDTheta.size() % 2 == 0)
            medianDTheta = (allDTheta.get(allDTheta.size()/2) + (allDTheta.get(allDTheta.size()/2-1)))/2;
        else
            medianDTheta = (allDTheta.get(allDTheta.size()/2));


        if (allDist.size() % 2 == 0)
            medianDist = (allDist.get(allDist.size()/2) + (allDist.get(allDist.size()/2-1)))/2;
        else
            medianDist = (allDist.get(allDist.size()/2));

        telemetry.addData("dTheta Mean: ", meanDTheta);
        telemetry.addData("dist Mean: ", meanDist);

        telemetry.addData("dTheta Median: ", medianDTheta);
        telemetry.addData("dist Median: ", medianDist);

        telemetry.addData("dTheta Stand Dev: ", standDevDThetaSum);
        telemetry.addData("dist Stand Dev: ", standDevDistSum);

        telemetry.addData("Valid dTheta %: ", validCountDTheta);
        telemetry.addData("Valid dist %", validCountDist);

        int count = 0;
        for(Double dTheta: allDTheta){
            count++;
            telemetry.addData("dTheta #" + count + " Val: ", dTheta);
        }

        count = 0;
        for(Double dist: allDist){
            count++;
            telemetry.addData("dist #" + count + " Val: ", dist);
        }

        telemetry.update();

        robot.pause(100);






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
