package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLUE_ALLIANCE", group="Autonomous")
public class BLUE_ALLIANCE extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addData("Blue Side Initialize" , "");
        telemetry.update();

        waitForStart();

        PoseTransfer.isBlue=true;

    }

}
