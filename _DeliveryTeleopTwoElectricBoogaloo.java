package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@TeleOp(name = "DeliveryTeleopTwoElectricBoogaloo", group = "TeleOp")
public class _DeliveryTeleopTwoElectricBoogaloo extends LinearOpMode {

    enum MODE {
        MANUAL,
        AUTO
    }

    int idealGridCoordX = PoseTransfer.idealGridCoordX;

    int idealGridCoordY = PoseTransfer.idealGridCoordY;

    double idealGridAngle = PoseTransfer.idealGridAngle;

    double slidePos = 0;


    MODE currentMode = MODE.AUTO;

    public ArrayList<Double> dThetas = new ArrayList<>();
    public ArrayList<Double> dists = new ArrayList<>();

    double dTheta = 2;
    double dist = 0;

    @Override
    public void runOpMode(){

        Pose2d startPos = new Pose2d();
        if(PoseTransfer.isBlue){
            startPos = PoseTransfer.currentPose.plus(new Pose2d(0,0,Math.toRadians(180)));
        }
        else{
            startPos = PoseTransfer.currentPose.plus(new Pose2d(0,0,Math.toRadians(180)));
        }

        //0,0 = Blue right corner square
        //0,5 = Blue left corner square
        //0,5 = Red left corner square
        //5,5 = Red right corner square



        int[] validRobotPosConversion = new int[6];

        for(int i = 0; i< validRobotPosConversion.length; i++){
            validRobotPosConversion[i]= ((i*24) + 12) - 72;
        }



        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        robot.vision.activateYellowPipelineCamera1();
        robot.vision.activateAprilTagYellowPipelineCamera2();

        robot.vision.runAprilTag(false);

        //CameraThread cameraThread = new CameraThread(robot.sensors, robot.vision);

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.drive.setPoseEstimate(startPos);

        robot.sensors.setImuOffset(startPos.getHeading());

        double slow = 1;

        slidePos = PoseTransfer.slidePos;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;




        robot.delivery.initEncodersNoReset();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high










        waitForStart();

        //cameraThread.run();


        while(!isStopRequested() && opModeIsActive()) {

            robot.sensors.retractOdo();

            /*


            ----- DRIVER 1 (DRIVETRAIN) -----


             */

            /*telemetry.addData("Camera Thread Active: ", cameraThread.isStarted());
            telemetry.addData("Camera Thread Active dTheta: ", cameraThread.getDTheta());
            telemetry.addData("Camera Thread Active dist: ", cameraThread.getDist());*/

            telemetry.update();


            if (gamepad1.left_bumper) {
                slow = 2;
            } else {
                slow = 1;
            }

            //TODO: FIND OUT WHERE /SLOW GOES & TEST
            //TODO: HAVE CHECK AGAINST IMU FOR IF/WHEN "GET HEADING" IS NOT ACCURATE


            Vector2d input = new Vector2d();
            if(gamepad1.right_bumper){
                input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                );

            }
            else{
                input = new Vector2d(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y
                ).rotated(-robot.sensors.getIMUReadout());
            }



            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() / slow,
                            input.getY() / slow,
                            -gamepad1.right_stick_x / slow
                    )
            );

            telemetry.addData("IMU: ", robot.sensors.getIMUReadout());

            //robot.drive.update();


            //robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);

            if(gamepad1.y){
                robot.sensors.setImuOffset(-robot.sensors.getIMUReadout());
            }








            /*


            ----- DRIVER 2 (SLIDE) -----


             */


            if (gamepad2.dpad_up && !dpadUpHeld) {
                slidePosIdx++;
                dpadUpHeld = true;

                if(slidePosIdx>3){
                    slidePosIdx=3;
                }

                slidePos = robot.delivery.slideIdxToEncoderVal(slidePosIdx);
            } else if (gamepad2.dpad_down && !dpadDownHeld) {
                slidePosIdx--;
                dpadDownHeld = true;

                if(slidePosIdx<0){
                    slidePosIdx=0;
                }

                slidePos = robot.delivery.slideIdxToEncoderVal(slidePosIdx);
            }

            if(gamepad2.right_trigger>0.75){
                slidePos=500;
                slidePosIdx=0;
            }

            if (!gamepad2.dpad_up) {
                dpadUpHeld = false;
            }

            if (!gamepad2.dpad_down) {
                dpadDownHeld = false;
            }


            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                slidePos += -gamepad2.left_stick_y * 30;
            }



            robot.delivery.runSlide((int) (slidePos), 0.9);

            if (gamepad2.y) { //Reset button
                slidePos = 0;
                slidePosIdx = 0;
                robot.delivery.resetEncoders();
            }


            /*


            ----- LEDs -----


             */

            //double sensorDist = robot.sensors.getFrontDist();

            if(getSlidePos()>1250){

                double newDTheta = robot.vision.findClosePoleDTheta();
                double newDist = robot.vision.findClosePoleDist();


                if(dThetas.size()>5 && newDTheta!=-1){
                    dThetas.remove(0);
                    dThetas.add(newDTheta);
                }
                else if(newDTheta!=-1){
                    dThetas.add(newDTheta);
                }


                if(dists.size()>5 && newDist!=-1){
                    dists.remove(0);
                    dists.add(newDist);
                }
                else if(newDist!=-1){
                    dists.add(newDist);
                }


                //Take mean dTheta if not empty
                dTheta = 2;
                if(dThetas.size()!=0){
                    if (dThetas.size() % 2 == 0)
                        dTheta = (dThetas.get(dThetas.size()/2) + (dThetas.get(dThetas.size()/2-1)))/2;
                    else
                        dTheta = (dThetas.get(dThetas.size()/2));
                }


                //Take mean dist if not empty
                dist = 0;
                if(dists.size()!=0){
                    if (dists.size() % 2 == 0)
                        dist = (dists.get(dists.size()/2) + (dists.get(dists.size()/2-1)))/2;
                    else
                        dist = (dists.get(dists.size()/2));
                }




                if(isEqual(dTheta, Math.toRadians(20), 0) && isEqual(dist, 7, 6)){
                    robot.sensors.setLEDState(Sensors.LED_STATE.POLE_GOOD);
                    telemetry.addData("Pole Good! ", "");
                    gamepad1.rumble(1, 1, 500);
                    gamepad2.rumble(1, 1, 500);
                }
                else{
                    robot.sensors.setLEDState(Sensors.LED_STATE.POLE_BAD);
                    telemetry.addData("Pole Bad! ", "");
                    gamepad1.rumble(0,0,500);
                    gamepad1.rumble(0,0,500);
                }


                //pause(0.110);


            }
            else{
                dThetas.clear();
                dists.clear();
                robot.sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
                telemetry.addData("Slide Down! ", "");
            }

            if (gamepad2.a) {
                //robot.delivery.runGripper(0.225);
                robot.delivery.openGripper();
                double frontDist = robot.sensors.getFrontDist();
                telemetry.addData("Front Dist: ", frontDist);
                if(slidePos<1250 && frontDist<3){
                    gamepad2.rumble(1,1,500);
                    robot.sensors.setLEDState(Sensors.LED_STATE.CONE_DETECTED);
                }
            } else {
                //robot.delivery.runGripper(0.5);
                gamepad2.rumble(0,0,500);
                robot.delivery.closeGripper();
            }

            if(gamepad1.a){
                robot.delivery.openGripper();
            }


            telemetry.addData("median dTheta: ", dTheta);
            telemetry.addData("median dist: ", dist);
            //telemetry.addData("Sensor dist: ", sensorDist);
            telemetry.update();








        }

        //cameraThread.stop();

    }

    public boolean isEqual (double x, double delta, double a) //X = sensor input, A = ideal input, delta = range/2
    {
        return Math.abs(x-a) < (delta/2);
    }

    public double getSlidePos(){
        return slidePos;
    }

}


                    /*if(gamepad1.y){
                        int closestX = 0;
                        double closestTempValX = 100;
                        int closestY = 0;
                        double closestTempValY = 100;
                        int closestAngle = 0;
                        double closestTempValAngle = 500;
                        for(int i = 0; i< 6; i++){
                            if(Math.abs(validRobotPosConversion[i]-robot.drive.getPoseEstimate().getX()) < closestTempValX){
                                closestTempValX = robot.drive.getPoseEstimate().getX();
                                closestX = i;
                            }
                        }

                        for(int i = 0; i< 6; i++){
                            if(Math.abs(validRobotPosConversion[i]-robot.drive.getPoseEstimate().getY()) < closestTempValY){
                                closestTempValY = robot.drive.getPoseEstimate().getY();
                                closestY = i;
                            }
                        }

                        for(int i = 0; i<360; i+=90){
                            if(Math.abs(idealGridAngle - robot.drive.getPoseEstimate().getHeading()) < closestTempValAngle){
                                closestTempValAngle = robot.drive.getPoseEstimate().getHeading();
                                closestAngle = i;
                            }
                        }

                        idealGridCoordX = closestX;
                        idealGridCoordY = closestY;
                        idealGridAngle = closestAngle;

                        robot.drive.setPoseEstimate(new Pose2d(validRobotPosConversion[idealGridCoordX], validRobotPosConversion[idealGridCoordY], Math.toRadians(idealGridAngle)));
                    }*/
