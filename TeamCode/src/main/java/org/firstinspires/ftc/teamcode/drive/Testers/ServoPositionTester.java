package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class ServoPositionTester extends LinearOpMode {
    Servo servo;
    double pos=0;
    boolean toggle1 = true;
    boolean toggle2 = true;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "Servo");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                servo.setPosition(pos);
            }
            if(gamepad1.x) {
                if(toggle1) {
                    pos = pos + 0.025;
                    toggle1=false;
                }
                if(pos>1) pos =1;
            }
            else toggle1=true;

            if(gamepad1.y) {
                if(toggle2) {
                    pos = pos - 0.025;
                    toggle2=false;
                }
                if(pos<-1) pos=0;
            } else toggle2= true;

            telemetry.addData("current pos", pos);
            telemetry.update();
        }
    }
}
