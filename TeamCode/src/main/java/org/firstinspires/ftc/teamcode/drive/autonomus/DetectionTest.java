package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DetectionTest extends LinearOpMode {

    RedWebCam rcam = new RedWebCam();
    //BlueWebCam bcam = new BlueWebCam();
    String desiredColor= "red";

    @Override
    public void runOpMode() throws InterruptedException {

        rcam.init(hardwareMap);
        //bcam.init(hardwareMap);

        while(opModeInInit()) {
            /*if(gamepad1.b) {
                if(desiredColor=="red") {
                    desiredColor="blue";
                }
                else {
                    desiredColor="red";
                }
            }*/

            if(desiredColor=="red") {
                telemetry.addData("autoCase",rcam.getAutoCase());
                telemetry.addData("leftAvg", rcam.getLeftVal());
                telemetry.addData("middleAvg", rcam.getMiddleVal());
                telemetry.addData("rightAvg", rcam.getRightVal());
            }
            else {
                /*telemetry.addData("autoCase",bcam.getAutoCase());
                telemetry.addData("leftAvg", bcam.getLeftVal());
                telemetry.addData("middleAvg", bcam.getMiddleVal());
                telemetry.addData("rightAvg", bcam.getRightVal());*/
            }
            telemetry.addData("color search",desiredColor);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
        }

    }

}
