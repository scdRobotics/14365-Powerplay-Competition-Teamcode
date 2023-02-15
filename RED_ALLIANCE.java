package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RED_ALLIANCE", group="Autonomous")
public class RED_ALLIANCE extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addData("Red Side Initialize" , "");
        telemetry.update();

        waitForStart();

        PoseTransfer.isBlue=false;

    }

}
