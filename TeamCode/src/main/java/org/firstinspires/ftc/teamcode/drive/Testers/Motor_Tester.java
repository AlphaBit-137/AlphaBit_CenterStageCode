package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

@TeleOp
public class Motor_Tester extends LinearOpMode {

    public DcMotorEx motor;

    public Motor_Skeleton motor_motor = new Motor_Skeleton(motor);

    Gamepad gamepad;


    @Override
    public void runOpMode() throws InterruptedException {

        motor_motor.init(hardwareMap,"Turret", false, false);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
                motor_motor.SetPower(1);
            else if(gamepad1.dpad_down)
                motor_motor.SetPower(-1);
            else motor_motor.SetPower(0);

            double angle = Ticks_To_Degrees();

            telemetry.addData("angle", angle);
            telemetry.update();
        }


    }


    public double Ticks_To_Degrees()
    {
        return Math.abs(motor_motor.MotorCurrentPosition() / 1992.6 * 360);
    }
}
