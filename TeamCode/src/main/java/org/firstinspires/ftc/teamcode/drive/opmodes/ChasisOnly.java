package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;


@TeleOp
public class ChasisOnly extends LinearOpMode {

        Robot_Drive drive;
        Claw claw;

        @Override
        public void runOpMode() {

            drive = new Robot_Drive(hardwareMap,gamepad1);
            //claw = new Claw(hardwareMap, gamepad1, "Claw");

            waitForStart();

            while (opModeIsActive())
            {
                //claw.Run();
                drive.Run();
            }


        }

}
