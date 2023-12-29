package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;
import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Arm2;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class Second extends LinearOpMode {

    Robot_Drive drive;

    Intake intake;

    Arm arm1 = new Arm();

    Arm2 arm2 = new Arm2();

    Slider slider;

    droneLauncher launcher;

    Gyroscope gyro = new Gyroscope();

    Claw claw;


    @Override
    public void runOpMode() {

        //drive = new Robot_Drive(hardwareMap,gamepad1);

        //claw = new Claw(hardwareMap,gamepad1);

        //intake = new Intake(hardwareMap , gamepad1);

        //launcher = new droneLauncher(hardwareMap,gamepad1);

        //arm1.init(hardwareMap,gamepad1);

        gyro.Init(hardwareMap);

        //arm2.init(hardwareMap,gamepad1);

        //slider = new Slider(hardwareMap,gamepad1);


        waitForStart();

        while (opModeIsActive())
        {
            //claw.Run();
            //drive.Run();
            //arm1.update();
            //arm2.update();
            //launcher.Run();
            //intake.Run();
            //slider.Run();
            //telemetry.addData("arm1 motor pos", arm1.getArmPos());
            //telemetry.addData("slider motor pow", slider.getPower());
            gyro.updateOrientation();
            telemetry.addData("get heading", gyro.getHeading());
            telemetry.addData("get forward angle",gyro.getForwardAngle());
            telemetry.addData("get lateral angle", gyro.getLateralAngle());
            telemetry.update();
        }
    }
}
