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

    public  double Reference=2000;
    public static double CurrentPos;
    double calculated_pid_pow;
    double caluclated_mpid_pow;
    boolean pid=false;

    /*static double n = 15; 38 // to be determined
    static double ku=0.025; 0.0019 // to be determined

    static double tu = 1/n; 0.026315
    static double Kp = 0.6*ku;
    static double Ki = 1.2*ku/tu;
    static double Kd = 0.075*ku*tu;*/

    static double Kp = 0.0020;//0.0020 // 0.0032
    static double Ki = 0.00045;//0.000045 // 0.000074
    static double Kd = 0.0011;//0.0011 // 0.00052
    double mkp = 0.0001;
    double mki = 0.00000002;
    double mkd = 0.00000005;
    boolean toggle = true;
    boolean toggle2=true;
    boolean manual = true;
    double max_output=0.5;


    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap,gamepad1);
        arm.SetPidCoefs(Kp,Ki,Kd);
        arm.SetMPidCoefs(mkp,mki,mkd,125,125);
        waitForStart();
        while (opModeIsActive()) {
            /*CurrentPos = arm.getArmPos();

            if(gamepad1.a) {
                arm.setReference(CurrentPos);
            }
            if(manual) {
                arm.simpleUpdate();
            }
            else {
                arm.tuneUpdate(gamepad2.b);
            }

            if(gamepad1.dpad_down) {
                if(manual) {
                    arm.SetPower(0);
                }
                manual=!manual;
            }*/
            //arm.setReference();
            armUpdate();
            if(gamepad1.b) {
                if(toggle) {
                    toggle=false;
                    manual=!manual;
                }
            }
            else {
                toggle=true;
            }

            telemetry.addData("pid power", calculated_pid_pow);
            telemetry.addData("pid power again", arm.ArmMotor1.getPidPower(Reference));
            telemetry.addData("math abs", Math.abs(Reference-arm.getArmPos()));
            //telemetry.addData()

            telemetry.addData("Reference", Reference);
            telemetry.addData("Position", arm.getArmPos());
            telemetry.addData("Manual",manual);
            telemetry.addData("using pid",pid);
            telemetry.update();
        }
    }
    void armUpdate() {
        if(gamepad1.left_bumper) {
            arm.SetPower(0.25);
            Reference = arm.getArmPos(); 
        }
        else if(gamepad1.right_bumper) {
            arm.SetPower(-0.25);
            Reference = arm.getArmPos();
        }
        else {
            calculated_pid_pow = -arm.ArmMotor1.getPidPower(Reference);
            //caluclated_mpid_pow = arm.ArmMotor1.returnMpidPower(Reference, -1);
            //if(calculated_pow > max_output) calculated_pow = max_output;
            //else if(calculated_pow < -max_output) calculated_pow = -max_output;
            if(manual) {
                arm.SetPower(0);
                //calculated_pow = -arm.ArmMotor1.getPidPower(Reference);
            }
            else {
                //calculated_pow = -arm.ArmMotor1.getPidPower(Reference);
                arm.SetPower(calculated_pid_pow);
                /*if(Math.abs(Reference-arm.getArmPos())<200 && Math.abs(Reference-arm.getArmPos())>190) {
                    arm.SetPower(0);
                }
                else if (Math.abs(Reference - arm.getArmPos()) > 200) {
                    toggle2=true;
                    arm.SetPower(caluclated_mpid_pow);
                    pid=false;
                } else {
                    arm.SetPower(calculated_pid_pow);
                    pid = true;
                }*/
            }
        }
    }

    double addons(double pow)
    {

        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;
        return pow;
    }

}
