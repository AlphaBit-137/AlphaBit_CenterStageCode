package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DetectionTest extends LinearOpMode {

    RedWebCam cam = new RedWebCam();

    @Override
    public void runOpMode() throws InterruptedException {

        cam.init(hardwareMap);

        while(opModeInInit()) {
            telemetry.addData("autoCase",cam.getAutoCase());
            telemetry.addData("leftAvg", cam.getLeftVal());
            telemetry.addData("middleAvg", cam.getMiddleVal());
            telemetry.addData("rightAvg", cam.getRightVal());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
        }

    }

}
