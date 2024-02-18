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

public class Outtake {

    DcMotorEx sucker1;
    DcMotorEx sucker2;

    Servo pick;

    Gamepad intake_gamepad;

    double suck_power = 0.55;

    double closed_pos = 0.3;
    double open_pos = 0.515;
    boolean open = false;

    ElapsedTime timer = new ElapsedTime();

    boolean rolling = false;
    boolean inversed = false;

    boolean toggle = true;
    boolean toggle2 = true;

    public Outtake(HardwareMap hwmap, Gamepad intake_gamepad)
    {
        sucker1 = hwmap.get(DcMotorEx.class,"Sucker1");
        sucker2 = hwmap.get(DcMotorEx.class,"Sucker2");

        sucker1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sucker2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sucker1.setDirection(DcMotorSimple.Direction.FORWARD);
        sucker2.setDirection(DcMotorSimple.Direction.FORWARD);

        pick = hwmap.get(Servo.class,"Pick");

        this.intake_gamepad = intake_gamepad;
    }

    public void Run()
    {
        if(intake_gamepad.b) {
            if(toggle) {
                if(open) {
                    pick.setPosition(closed_pos);
                }
                else {
                    pick.setPosition(open_pos);
                }
                open = !open;
                toggle=false;
            }
        }
        else {
            toggle=true;
        }

        if(intake_gamepad.y) {
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
            suck(-suck_power,-suck_power);
        }
        else {
            suck(0,0);
        }/*
        if(intake_gamepad.dpad_left) {
            suck(suck_power, suck_power);
        }
        else if(intake_gamepad.dpad_right) {
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
