package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PoseTransfer.alt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;


@TeleOp(name = "RED_GRID_TELEOP", group = "TeleOp")
public class _GRID_TELEOP extends LinearOpMode {

    enum MODE {
        MANUAL,
        AUTO
    }

    int idealXGridCord = 0;
    int idealYGridCord = 0;

    double idealAngle = PoseTransfer.currentPose.getHeading();


    MODE currentMode = MODE.AUTO;

    @Override
    public void runOpMode(){






        //0,0 = Blue right corner square
        //0,5 = Blue left corner square
        //0,5 = Red left corner square
        //5,5 = Red right corner square

        int[][][] validRobotPos = new int[6][6][2];

        for(int i = 0; i< validRobotPos.length; i++){
            for(int j = 0; j< validRobotPos.length; j++){
                validRobotPos[i][j][0]= ((i*24) + 12) - 72;
                validRobotPos[i][j][1]= ((i*24) + 12) - 72;
            }
        }

        Pose2d idealPose = new Pose2d(0 , 0, Math.toRadians(idealAngle));


        //TODO: Use new (much simpler) PoseTransfer.java to transfer valid coordinates from auto to teleop
        if(PoseTransfer.alliance=="BLUE" && PoseTransfer.currentPose.getX()>0){
            if(!PoseTransfer.alt){
                switch(PoseTransfer.park){
                    case 1:
                        idealXGridCord = 5;
                        idealYGridCord = 2;
                        idealPose=new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 4;
                        idealYGridCord = 2;
                        idealPose=new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 3;
                        idealYGridCord = 2;
                        idealPose=new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
            else{
                switch(PoseTransfer.park) {
                    case 1:
                        idealXGridCord = 5;
                        idealYGridCord = 1;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 4;
                        idealYGridCord = 1;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 3;
                        idealYGridCord = 1;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
        }
        else if(PoseTransfer.alliance=="BLUE" && PoseTransfer.currentPose.getX()<0){
            if(!PoseTransfer.alt){
                switch(PoseTransfer.park){
                    case 1:
                        idealXGridCord = 3;
                        idealYGridCord = 2;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 2;
                        idealYGridCord = 2;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 0;
                        idealYGridCord = 2;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
        }



        else if(PoseTransfer.alliance=="RED" && PoseTransfer.currentPose.getX()>0){
            if(!PoseTransfer.alt){
                switch(PoseTransfer.park){
                    case 1:
                        idealXGridCord = 3;
                        idealYGridCord = 3;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 4;
                        idealYGridCord = 3;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 5;
                        idealYGridCord = 3;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
            else{
                switch(PoseTransfer.park) {
                    case 1:
                        idealXGridCord = 3;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 4;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 5;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
        }

        else if(PoseTransfer.alliance=="RED" && PoseTransfer.currentPose.getX()<0){
            if(!PoseTransfer.alt) {
                switch (PoseTransfer.park) {
                    case 1:
                        idealXGridCord = 0;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 1;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 2;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
            else{
                switch (PoseTransfer.park) {
                    case 1:
                        idealXGridCord = 0;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 2:
                        idealXGridCord = 1;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                    case 3:
                        idealXGridCord = 2;
                        idealYGridCord = 4;
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));
                        break;
                }
            }
        }


        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, hardwareMap, telemetry, timer, false);

        Delivery delivery = robot.delivery;

        double slow = 1;

        double slidePos = 500;

        boolean dpadUpHeld = false;

        boolean dpadDownHeld = false;




        robot.drive.setPoseEstimate(PoseTransfer.currentPose);




        delivery.initEncodersNoReset();

        int slidePosIdx = 0; //0 = bottom, 1 = low, 2 = med, 3 = high



        TrajectorySequence goToFirstPoint = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())

                .lineToLinearHeading(idealPose)

                .build();


        waitForStart();

        robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.drive.followTrajectorySequenceAsync(goToFirstPoint);


        while(!isStopRequested() && opModeIsActive()){






            switch(currentMode){
                case AUTO:

                    if(gamepad1.a){
                        robot.drive.breakFollowing();
                        currentMode = MODE.MANUAL;
                    }

                    if(!robot.drive.isBusy()){
                        robot.drive.setPoseEstimate(idealPose);
                        currentMode= MODE.MANUAL;
                    }

                    robot.drive.update();

                    break;


                case MANUAL:

                    telemetry.addData("Ideal X Coord Grid: ", idealXGridCord);
                    telemetry.addData("Ideal Y Coord Grid: ", idealYGridCord);
                    telemetry.addData("Ideal Coord X: ", validRobotPos[idealXGridCord][idealYGridCord][0]);
                    telemetry.addData("Ideal Coord Y: ", validRobotPos[idealXGridCord][idealYGridCord][1]);
                    telemetry.addData("Ideal Heading: ", validRobotPos[idealXGridCord][idealYGridCord][1]);
                    telemetry.update();

                    Pose2d poseEstimate = robot.drive.getPoseEstimate();

                    if(gamepad1.dpad_up){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealYGridCord--;
                        }
                        else{
                            idealYGridCord++;
                        }

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }
                    else if(gamepad1.dpad_right){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealXGridCord--;
                        }
                        else{
                            idealXGridCord++;
                        }

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;
                    }
                    else if(gamepad1.dpad_down){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealYGridCord++;
                        }
                        else{
                            idealYGridCord--;
                        }

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }
                    else if(gamepad1.dpad_left){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealXGridCord++;
                        }
                        else{
                            idealXGridCord--;
                        }

                        checkBoundaries();

                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }

                    if(gamepad1.b){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealAngle-=90;
                        }
                        else{
                            idealAngle+=90;
                        }

                        checkBoundaries();

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .turn(Math.toRadians(idealAngle - robot.drive.getPoseEstimate().getHeading()))
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;


                    }
                    else if(gamepad1.x){
                        if(PoseTransfer.alliance=="BLUE"){
                            idealAngle+=90;
                        }
                        else{
                            idealAngle-=90;
                        }

                        checkBoundaries();

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .turn(Math.toRadians(idealAngle - robot.drive.getPoseEstimate().getHeading()))
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;

                    }

                    if(gamepad1.left_stick_button){
                        idealPose = new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle));

                        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                .lineToLinearHeading(idealPose)
                                .build();

                        robot.drive.followTrajectorySequenceAsync(traj);

                        currentMode = MODE.AUTO;
                    }


                    if(gamepad1.left_bumper && (gamepad1.right_stick_x>0.1 || gamepad1.right_stick_x<-0.1)){
                        slow=5;
                    }
                    else if(gamepad1.left_bumper && (-gamepad1.left_stick_y>0.1 || gamepad1.left_stick_x>0.1 || gamepad1.left_stick_x<-0.1 || -gamepad1.left_stick_y<-0.1)){
                        slow=2;
                    }
                    else{
                        slow=1;
                    }

                    if(gamepad1.a){
                        //TODO: AUTONOMOUS POLE movements (should be straightforward. Mostly copied from auto.

                        if(gamepad1.left_stick_x>0.25 && -gamepad1.left_stick_y>0.25){
                            if(PoseTransfer.alliance=="BLUE"){
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(-45))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                            else{
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(45))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                        }
                        else if(gamepad1.left_stick_x<-0.25 && -gamepad1.left_stick_y>0.25){
                            if(PoseTransfer.alliance=="BLUE"){
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(45))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                            else{
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(-45))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                        }
                        else if(gamepad1.left_stick_x<-0.25 && -gamepad1.left_stick_y<-0.25){
                            if(PoseTransfer.alliance=="BLUE"){
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(-135))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                            else{
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(135))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                        }
                        else if(gamepad1.left_stick_x>0.25 && -gamepad1.left_stick_y<-0.25){
                            if(PoseTransfer.alliance=="BLUE"){
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(135))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                            else{
                                TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(poseEstimate)
                                        .turn(Math.toRadians(-135))
                                        .forward(8)
                                        .build();

                                robot.drive.followTrajectorySequenceAsync(traj);
                                currentMode = MODE.AUTO;
                            }
                        }

                    }
                    else{
                        //MANUAL movements
                        robot.drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y/slow,
                                        gamepad1.left_stick_x/slow,
                                        -gamepad1.right_stick_x/slow
                                )
                        );
                    }

                    if(gamepad1.y){
                        int closestX = 0;
                        double closestTempValX = 100;
                        int closestY = 0;
                        double closestTempValY = 100;
                        int closestAngle = 0;
                        double closestTempValAngle = 500;
                        for(int i = 0; i< 6; i++){
                            if(Math.abs(validRobotPos[i][0][0]-robot.drive.getPoseEstimate().getX()) < closestTempValX){
                                closestTempValX = robot.drive.getPoseEstimate().getX();
                                closestX = i;
                            }
                        }

                        for(int i = 0; i< 6; i++){
                            if(Math.abs(validRobotPos[0][i][0]-robot.drive.getPoseEstimate().getY()) < closestTempValY){
                                closestTempValY = robot.drive.getPoseEstimate().getY();
                                closestY = i;
                            }
                        }

                        for(int i = 0; i<360; i+=90){
                            if(Math.abs(idealAngle - robot.drive.getPoseEstimate().getHeading()) < closestTempValAngle){
                                closestTempValAngle = robot.drive.getPoseEstimate().getHeading();
                                closestAngle = i;
                            }
                        }

                        idealXGridCord = closestX;
                        idealYGridCord = closestY;
                        idealAngle = closestAngle;

                        robot.drive.setPoseEstimate(new Pose2d(validRobotPos[idealXGridCord][idealYGridCord][0], validRobotPos[idealXGridCord][idealYGridCord][1], Math.toRadians(idealAngle)));


                    }


                    break;




            }








            /*

                            (0)--------------(Y)--------------(5)
            (0,0)
                            /-----------------------------------\                    (0)
                            |     *     *     *     *     *     |                     |
                            |*****o*****o*****o*****o*****o*****|                     |
                            |     *     *     *     *     *     |                     |                             1
                            |*****O*****o*****O*****o*****O*****|                     |                             ^
                            |     *     *     *     *     *     |                     |                             |
              Blue Side     |*****O*****o*****O*****o*****O*****|     Red Side       (X)                2 <-- (Orientation) --> 0
                            |     *     *     *     *     *     |                     |                             |
                            |*****o*****o*****o*****o*****o*****|                     |                             ∨
                            |     *     *     *     *     *     |                     |                             3
                            |*****O*****o*****O*****o*****O*****|                     |
                            |     *     *     *     *     *     |                     |
                            \-----------------------------------/                    (5)
                                                                         (5,5)

             */

            /*double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;*/




            /*if(!robot.drive.isBusy()){
                if(gamepad1.a){
                    if(-y>0.15 && x>0.15){ //Flicked Up Right
                        robot.drive.followTrajectorySequence(upRight);
                    }
                    else if(-y>0.15 && x<-0.15){ //Flicked Up Left
                        robot.drive.followTrajectorySequence(upLeft);
                    }
                    else if(-y<-0.15 && x<-0.15){ //Flicked Down Left
                        robot.drive.followTrajectorySequence(downLeft);
                    }
                    else if(-y<-0.15 && x>0.15){ //Flicked Down Right
                        robot.drive.followTrajectorySequence(downRight);
                    }
                }

                else{
                    if(-y>0.075 || x>0.0825 || rx>0.075 || -y<-0.075 || x<-0.0825 || rx<-0.075){
                        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.drive.setMotorPowers(frontLeftPower/slow, backLeftPower/slow, backRightPower/slow, frontRightPower/slow);
                    }
                    else{
                        robot.drive.setMotorPowers(0,0,0,0);
                    }
                }
            }

            if(gamepad1.dpad_up){
                robot.drive.followTrajectorySequence(forwardGrid);
            }
            else if(gamepad1.dpad_right){
                robot.drive.followTrajectorySequence(strafeRightGrid);
            }
            else if(gamepad1.dpad_left){
                robot.drive.followTrajectorySequence(strafeLeftGrid);
            }
            else if(gamepad1.dpad_down){
                robot.drive.followTrajectorySequence(backwardGrid);
            }

            if(gamepad1.b){
                robot.drive.followTrajectorySequence(turnRight);
            }
            else if(gamepad1.x){
                robot.drive.followTrajectorySequence(turnLeft);
            }*/


            /*if(gamepad1.y){
                robot.drive.setPoseEstimate(new Pose2d(0,0,0));
                robot.drive.breakFollowing();
                robot.drive.setDrivePower(new Pose2d());
            }


            robot.drive.update();*/









            if(gamepad2.dpad_up && !dpadUpHeld){
                slidePosIdx++;
                dpadUpHeld=true;

                slidePos = delivery.slideIdxToEncoderVal(slidePosIdx);
            }
            else if(gamepad2.dpad_down && !dpadDownHeld){
                slidePosIdx--;
                dpadDownHeld=true;

                slidePos = delivery.slideIdxToEncoderVal(slidePosIdx);
            }

            if(!gamepad2.dpad_up){
                dpadUpHeld=false;
            }
            if(!gamepad2.dpad_down){
                dpadDownHeld=false;
            }





            if(gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1){
                slidePos += -gamepad2.left_stick_y*10;
            }


            delivery.runSlide((int) (slidePos), 0.9);


            if(gamepad2.a){
                delivery.runGripper(0.225);
            }
            else{
                delivery.runGripper(0);
            }

            if(gamepad2.y){ //Reset button
                slidePos=0;
                slidePosIdx=0;
                delivery.resetEncoders();
            }




            delivery.getEncoderValues();



        }

    }

    public void checkBoundaries(){
        if(idealAngle>360){
            idealAngle-=360;
        }
        else if(idealAngle<0){
            idealAngle+=360;
        }

        if(idealXGridCord>5){
            idealXGridCord=5;
        }
        else if(idealXGridCord<0){
            idealXGridCord=0;
        }

        if(idealYGridCord>5){
            idealYGridCord=5;
        }
        else if(idealYGridCord<0){
            idealYGridCord=0;
        }
    }

}
