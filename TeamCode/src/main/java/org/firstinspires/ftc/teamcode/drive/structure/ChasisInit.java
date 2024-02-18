package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

public class ChasisInit {

    public DcMotorEx BackLeft = null;
    public DcMotorEx FrontRight = null;

    public DcMotorEx FrontLeft = null;
    public DcMotorEx BackRight = null;

    HardwareMap hwMap = null;

    public ChasisInit(HardwareMap ahwMap){

        hwMap = ahwMap;

        BackLeft = hwMap.get(DcMotorEx.class, "Back_Left");
        FrontRight = hwMap.get(DcMotorEx.class, "Front_Right");
        FrontLeft = hwMap.get(DcMotorEx.class, "Front_Left");
        BackRight = hwMap.get(DcMotorEx.class, "Back_Right");


        ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>(Arrays.asList(BackLeft, FrontLeft, FrontRight, BackRight));

        for (DcMotorEx motor : motors) {

          motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

          motor.setPower(0);

          motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

          motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        BackRight.setDirection(DcMotorEx.Direction.FORWARD);

    }

        public void  setMotorPower (double BackLeftPower ,double FrontRightPower,double FrontLeftPower,double BackRightPower)
    {
        BackLeft.setPower(BackRightPower);
        FrontRight.setPower(FrontRightPower);
        FrontLeft.setPower(FrontLeftPower);
        BackRight.setPower(BackLeftPower);
    }

}
