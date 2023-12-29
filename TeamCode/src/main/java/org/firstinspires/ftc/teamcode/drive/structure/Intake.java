package org.firstinspires.ftc.teamcode.drive.structure;

import android.icu.lang.UScript;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    DcMotorEx sucker1;
    DcMotorEx sucker2;

    Gamepad shooters_gamepad;

    double suck_power = 0.55;
    ElapsedTime timer = new ElapsedTime();

    boolean rolling = false;
    boolean inversed = false;

    boolean toggle = true;
    boolean toggle2 = true;

    public Intake(HardwareMap hwmap, Gamepad shooters_gamepad)
    {
        sucker1 = hwmap.get(DcMotorEx.class,"Sucker1");
        sucker2 = hwmap.get(DcMotorEx.class,"Sucker2");
        sucker1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sucker2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sucker1.setDirection(DcMotorSimple.Direction.FORWARD);
        sucker2.setDirection(DcMotorSimple.Direction.FORWARD);

        this.shooters_gamepad = shooters_gamepad;
    }

    public void Run()
    {
        if(shooters_gamepad.b) {
            if(toggle) {
                rolling=!rolling;
                toggle=false;
            }
        }
        else {
            toggle=true;
        }

        if(shooters_gamepad.y) {
            if(toggle2) {
                inversed=!inversed;
                toggle2=false;
            }
        }
        else {
            toggle2=true;
        }

        if(rolling)
        {
            suck(suck_power,suck_power);
        }
        else if(inversed)
        {
            suck(-suck_power+0.2,-suck_power);
        }
        else {
            suck(0,0);
        }/*
        if(shooters_gamepad.dpad_left) {
            suck(suck_power, suck_power);
        }
        else if(shooters_gamepad.dpad_right) {
            suck(-suck_power+0.2, -suck_power);
        }
        else {
            suck(0,0);
        }*/
    }

    public void suck(double power1, double power2)
    {
        sucker1.setPower(power1);
        sucker2.setPower(power2);
    }

}
