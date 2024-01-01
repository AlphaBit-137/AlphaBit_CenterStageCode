package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
@TeleOp
public class ChasisDirectionTest extends LinearOpMode {
    Robot_Drive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        boolean b1 = false;
        boolean b2 = false;
        boolean b3 = false;
        boolean b4 = false;

        boolean t1 = false;
        boolean t2 = false;
        boolean t3 = false;
        boolean t4 = false;

        drive = new Robot_Drive(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                if(t1) {
                    t1=false;
                    b1=!b1;
                }
            }
            else {
                t1=true;
            }

            if(gamepad1.b) {
                if(t2) {
                    t2=false;
                    b2=!b2;
                }
            }
            else {
                t2=true;
            }

            if(gamepad1.x) {
                if(t3) {
                    t3=false;
                    b3=!b3;
                }
            }
            else {
                t3=true;
            }

            if(gamepad1.y) {
                if(t4) {
                    t4=false;
                    b4=!b4;
                }
            }
            else {
                t4=true;
            }

            if(b1) {
                drive.backLeftPow(0.5);
            }
            else drive.backLeftPow(0);

            if(b2) {
                drive.backRightPow(0.5);
            }
            else drive.backRightPow(0);

            if(b3) {
                drive.frontLeftPow(0.5);
            }
            else drive.frontLeftPow(0);

            if(b4) {
                drive.frontRightPow(0.5);
            }
            else drive.frontRightPow(0);

            telemetry.addData("backLeft",b1);
            telemetry.addData("backRight",b2);
            telemetry.addData("frontLeft",b3);
            telemetry.addData("frontRight",b4);
            telemetry.update();
        }
    }
}
