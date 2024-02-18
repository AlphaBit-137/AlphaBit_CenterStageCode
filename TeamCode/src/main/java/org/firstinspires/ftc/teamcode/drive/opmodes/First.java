package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Outtake;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.Slider2;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {

    Robot_Drive drive;
    droneLauncher launcher;
    Slider slider1;
    Slider2 slider2;
    Intake in;
    Outtake out;


    @Override
    public void runOpMode() {

        drive = new Robot_Drive(hardwareMap,gamepad1);
        launcher = new droneLauncher(hardwareMap,gamepad1);
        slider1 = new Slider(hardwareMap,gamepad2);
        slider2 = new Slider2(hardwareMap, gamepad2);
        in = new Intake(hardwareMap, gamepad2);
        out = new Outtake(hardwareMap, gamepad2);

        waitForStart();

        while (opModeIsActive())
        {
             drive.Run();
             launcher.Run();
             slider1.Run();
             slider1.Run();
             in.Run();
             out.Run();

             telemetry.update();
        }
    }
}
