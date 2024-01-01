package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;


@TeleOp
public class ChasisOnly extends LinearOpMode {

        Robot_Drive drive;
        Claw claw;
        DcMotorEx parallel;
        DcMotorEx perpen;

        @Override
        public void runOpMode() {

            drive = new Robot_Drive(hardwareMap,gamepad1);
            parallel = hardwareMap.get(DcMotorEx.class,"parallelEncoder");
            perpen = hardwareMap.get(DcMotorEx.class,"perpendicularEncoder");
            //claw = new Claw(hardwareMap, gamepad1, "Claw");

            waitForStart();

            while (opModeIsActive())
            {
                //claw.Run();
                drive.Run();
                telemetry.addData("para",parallel.getCurrentPosition());
                telemetry.addData("perpen",perpen.getCurrentPosition());
                telemetry.update();
            }


        }

}
