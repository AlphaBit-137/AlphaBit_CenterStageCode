package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Arm2;

import java.lang.ref.Reference;

@Config
@TeleOp
public class PID_Tuning extends LinearOpMode {

    public static double Reference;
    public static double CurrentPos;
    static double ku=0.025;
    static double Kp = 0.007;
    static double Ki = 0.0;
    static double Kd = 0.001;
    public static double kP=0.05;
    public static  double kI=0;
    public static double kD=0;
    Arm arm = new Arm();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.SetPidCoefs(Kp,Ki,Kd);
        arm.init(hardwareMap,gamepad1);
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
