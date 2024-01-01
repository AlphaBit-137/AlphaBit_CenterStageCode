package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {

    Robot_Drive drive;

    Arm arm1;

    Claw claw;

    droneLauncher launcher;


    @Override
    public void runOpMode() {

        drive = new Robot_Drive(hardwareMap,gamepad1);

        claw = new Claw(hardwareMap,gamepad1);

        //launcher = new droneLauncher(hardwareMap,gamepad1);

        arm1 = new Arm(hardwareMap,gamepad1);


        waitForStart();

        while (opModeIsActive())
        {
             drive.Run();
             arm1.update();
             //arm2.update();
             //launcher.Run();
            claw.Run();
             //telemetry.addData("arm1 motor pos", arm1.getArmPos());
             telemetry.addData("arm motor pos", arm1.getArmPos());
             telemetry.update();
        }
    }
}
