package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class ServoPositionTester extends LinearOpMode {
    Servo servo;
    Slider slider;
    double pos=0;
    boolean toggle1 = true;
    boolean toggle2 = true;
    @Override
    public void runOpMode() throws InterruptedException {
        slider = new Slider(hardwareMap,gamepad1);
        servo = hardwareMap.get(Servo.class, "Claw2");

        waitForStart();

        while (opModeIsActive()) {
            slider.Run();
            if(gamepad1.a) {
                servo.setPosition(pos);
            }
            if(gamepad1.x) {
                if(toggle1) {
                    pos = pos + 0.05;
                    toggle1=false;
                }
                if(pos>1) pos =1;
            }
            else toggle1=true;

            if(gamepad1.y) {
                if(toggle2) {
                    pos = pos - 0.05;
                    toggle2=false;
                }
                if(pos<-1) pos=0;
            } else toggle2= true;

            telemetry.addData("current pos", pos);
            telemetry.update();
        }
    }
}
