package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;

@Autonomous
public class ParkAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    Robot_Drive drive;
    Arm arm;
    Claw claw;
    static double Kp = 0.0025;
    static double Ki = 0.0;
    static double Kd = 0.001025;
    @Override
    public void runOpMode() throws InterruptedException {

        claw = new Claw(hardwareMap,gamepad1);
        drive = new Robot_Drive(hardwareMap,gamepad1);
        arm = new Arm(hardwareMap,gamepad1);
        armInit();

        while (!opModeInInit()) {
            telemetry.addLine("goes backwards to the parking zone");
            telemetry.update();
        }
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if(timer.seconds()<2.7) {
                armUpdate();
                drive.MS(-0.3,-0.3,-0.3,-0.3);
            }
            else {
                drive.MS(0,0,0,0);
                arm.SetPower(0);
            }
        }
    }

    void armUpdate() {
            arm.SetPower(-arm.ArmMotor1.getPidPower(250
            ));
    }

    void armInit() {
        arm.SetPidCoefs(Kp,Ki,Kd);
    }
}
