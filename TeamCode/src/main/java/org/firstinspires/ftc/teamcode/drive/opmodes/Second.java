package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;
import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Claw;
import org.firstinspires.ftc.teamcode.drive.structure.Robot_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.droneLauncher;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class Second extends LinearOpMode {

    Robot_Drive drive;

    Intake intake;

    Arm arm1;

    Slider slider;

    droneLauncher launcher;

    Gyroscope gyro = new Gyroscope();

    Claw claw;

    public double encoder_pos;

    double Reference = 200;
    double calculated_pow=0;
    boolean manual=true;
    boolean toggle=true;

    double max_output=0.7;

    static double Kp = 0.0025;
    static double Ki = 0.0;
    static double Kd = 0.001025; //0.0005


    @Override
    public void runOpMode() {

        drive = new Robot_Drive(hardwareMap,gamepad1);

        claw = new Claw(hardwareMap,gamepad1);

        launcher = new droneLauncher(hardwareMap,gamepad1);

        arm1 = new Arm(hardwareMap,gamepad1);
        armInit();

        //gyro.Init(hardwareMap);

        //arm2.init(hardwareMap,gamepad1);

        //slider = new Slider(hardwareMap,gamepad1);


        waitForStart();

        while (opModeIsActive())
        {

            armUpdate();
            drive.Run();
            claw.Run();
            launcher.Run();

            if(gamepad1.b) {
                if(toggle) {
                    toggle=false;
                    manual=!manual;
                }
            }
            else {
                toggle=true;
            }
            if(gamepad1.dpad_up) {
                Reference=2700;
            }
            else if(gamepad1.dpad_left) {
                Reference=700;
            }
            else if(gamepad1.dpad_down) {
                Reference=200;
            }

            telemetry.addData("arm pow", arm1.pow);
            telemetry.addData("arm pos",arm1.getArmPos());

            telemetry.update();
        }
    }

    void armUpdate() {
        if(gamepad1.left_bumper) {
            arm1.SetPower(0.25);
            Reference = arm1.getArmPos();
        }
        else if(gamepad1.right_bumper) {
            arm1.SetPower(-0.25);
            Reference = arm1.getArmPos();
        }
        else {
            calculated_pow = -arm1.ArmMotor1.getPidPower(Reference);
            //if(calculated_pow > max_output) calculated_pow = max_output;
            //else if(calculated_pow < -max_output) calculated_pow = -max_output;
            if(manual) {
                arm1.SetPower(0);
                //calculated_pow = -arm.ArmMotor1.getPidPower(Reference);
            }
            else {
                //calculated_pow = -arm.ArmMotor1.getPidPower(Reference);
                arm1.SetPower(calculated_pow);
            }
        }
    }

    void armInit() {
        arm1.SetPidCoefs(Kp,Ki,Kd);
    }

    double addons(double pow)
    {

        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;
        return pow;
    }
}
