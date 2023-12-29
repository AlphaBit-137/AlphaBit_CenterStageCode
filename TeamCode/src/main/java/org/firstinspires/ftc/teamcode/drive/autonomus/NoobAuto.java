package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class NoobAuto extends LinearOpMode {

    WebCam camera = new WebCam();
    Robot_Drive drive;

    int autoCase;

    @Override
    public void runOpMode() throws InterruptedException {

        camera.init(hardwareMap);
        drive = new Robot_Drive(hardwareMap,gamepad1);
        autoCase=camera.getAutoCase();

        while(opModeInInit()&&!isStopRequested()) {
            telemetry.addData("autoCase",autoCase);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            drive.goForward(2,0.4);

            switch (autoCase) {
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }

            telemetry.update();
        }

    }

}
