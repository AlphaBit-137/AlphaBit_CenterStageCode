package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.FieldCentric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;


@TeleOp
public class ChasisOnly extends LinearOpMode {

        Robot_Drive robo_drive;
        FieldCentric_Drive field_drive;
        Claw claw;
        Arm arm;
        droneLauncher launcher;
        DcMotorEx parallel;
        DcMotorEx perpen;
        boolean field_centric=false;
        double Kp = 0.002;
        double Ki = 0.0;
        double Kd = 0.0005;
        boolean toggle = true;

        @Override
        public void runOpMode() {

            //launcher = new droneLauncher(hardwareMap,gamepad1);
            robo_drive = new Robot_Drive(hardwareMap,gamepad1);
            field_drive = new FieldCentric_Drive(hardwareMap,gamepad1);
            //parallel = hardwareMap.get(DcMotorEx.class,"parallelEncoder");
            //perpen = hardwareMap.get(DcMotorEx.class,"perpendicularEncoder");
            arm = new Arm(hardwareMap,gamepad1);
            claw = new Claw(hardwareMap, gamepad1);

            waitForStart();
            armInit();
            while (opModeIsActive())
            {
                //armUpdate();
                //claw.Run();
                //launcher.Run();

                if(gamepad1.b) {
                    if(toggle) {
                        field_centric = !field_centric;
                        toggle= false;
                    }
                }
                else {
                    toggle=true;
                }

                if(field_centric) {
                    field_drive.Run();
                }
                else {
                    robo_drive.Run();
                }
                //robo_drive.Run();
                //field_drive.Run();
                //telemetry.addData("para",parallel.getCurrentPosition());
                //telemetry.addData("perpen",perpen.getCurrentPosition());
                telemetry.addData("field centric",field_centric);
                telemetry.update();
            }


        }

    void armUpdate() {
        arm.SetPower(-arm.ArmMotor1.getPidPower(300));
    }

    void armInit() {
        arm.SetPidCoefs(Kp,Ki,Kd);
    }

}
