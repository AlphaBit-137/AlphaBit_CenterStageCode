package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.structure.Claw;

@Autonomous
public class DummyAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw;
        claw=new Claw(hardwareMap,gamepad1);
        while (opModeInInit()) {
            telemetry.addLine("dummy auto, doesnt do anything");
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            //nuthin
        }
    }
}
