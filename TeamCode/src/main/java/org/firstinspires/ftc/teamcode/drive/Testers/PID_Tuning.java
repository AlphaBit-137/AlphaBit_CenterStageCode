package org.firstinspires.ftc.teamcode.drive.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;

import java.lang.ref.Reference;

@Config
@TeleOp
public class PID_Tuning extends LinearOpMode {

    public static double Reference;
    public static double CurrentPos;

    static double n = 15; // to be determined
    static double ku=0.025; // to be determined

    static double tu = 1/n;
    static double Kp = 0.6*ku;
    static double Ki = 1.2*ku/tu;
    static double Kd = 0.075*ku*tu;
    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.SetPidCoefs(Kp,Ki,Kd);
        arm = new Arm(hardwareMap,gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            CurrentPos = arm.getArmPos();
            if(gamepad1.a) {
                arm.setReference(CurrentPos);
            }
            arm.tuneUpdate(gamepad1.b);
            telemetry.addData("pid power", arm.getCalculatedPow());
            telemetry.addData("Reference", Reference);
            telemetry.addData("Position", CurrentPos);
            telemetry.update();
        }
    }
}
