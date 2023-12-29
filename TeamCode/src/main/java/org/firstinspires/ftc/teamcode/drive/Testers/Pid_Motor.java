package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
*
*Experimental class used to calculate the power returned to the motor using PID
*
 */

@TeleOp
public class Pid_Motor extends LinearOpMode {

    public DcMotorEx TestMotor;
    public DcMotorEx TestMotor2;

    @Override
    public void runOpMode() throws InterruptedException {

        TestMotor = hardwareMap.get(DcMotorEx.class,"Motor1");

        TestMotor2 = hardwareMap.get(DcMotorEx.class, "Motor2");

        TestMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TestMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TestMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        TestMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        TestMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TestMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a)
            {

                TestMotor.setPower(-0.6);

                TestMotor2.setPower(-1);

            }else if(gamepad1.b)
            {

                TestMotor.setPower(0.6);

                TestMotor2.setPower(1);

            }
            else
            {

                TestMotor.setPower(-0.1);

                TestMotor2.setPower(0);



            }

            if(gamepad1.dpad_up)
            {
                TestMotor.setVelocity(100);
            }

            if(gamepad1.dpad_down)
            {
                TestMotor.setVelocity(50);
            }

            if(gamepad1.dpad_right)
            {
                TestMotor.setVelocity(25);
            }

          //  TestMotor.setPower(1);
           // TestMotor.setPower(1);

            telemetry.addData("Motor Power", TestMotor.getPower());

            telemetry.addData("Motor position", TestMotor.getCurrentPosition());


            telemetry.addData("Motor2 position", TestMotor2.getCurrentPosition());

            telemetry.addData("Motor2 Power" , TestMotor2.getPower());

            telemetry.update();

        }
        }


    }

