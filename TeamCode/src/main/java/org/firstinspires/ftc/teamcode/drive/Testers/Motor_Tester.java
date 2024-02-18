package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

@TeleOp
public class Motor_Tester extends LinearOpMode {

    public DcMotorEx motor;
    public DcMotorEx motor2;

    double pow = 0.5;

    boolean toggle1 = true;
    boolean toggle2 = true;

    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class,"Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "Motor2");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.x) {
                if(toggle1 && pow <= 1) {
                    pow += 0.05;
                    toggle1 = false;
                }
            }
            else {
                toggle1 = true;
            }

            if(gamepad1.y) {
                if(toggle2 && pow > 0) {
                    pow -= 0.05;
                    toggle2 = false;
                }
            }
            else {
                toggle2 = true;
            }

            if(gamepad1.right_bumper) {
                motor.setPower(pow);
                motor2.setPower(-pow);
            }
            else if(gamepad1.left_bumper) {
                motor.setPower(-pow);
                motor2.setPower(pow);
            }
            else {
                motor.setPower(0);
                motor2.setPower(0);
            }

            telemetry.addData("Power: ", pow);
            telemetry.addData("Location1: ",motor.getCurrentPosition());
            telemetry.addData("Location2: ",motor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
