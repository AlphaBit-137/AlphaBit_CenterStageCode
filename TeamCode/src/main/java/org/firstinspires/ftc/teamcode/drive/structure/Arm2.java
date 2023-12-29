package org.firstinspires.ftc.teamcode.drive.structure;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Arm2 {

    public DcMotorEx arm2;

    public Motor_Skeleton ArmMotor2 = new Motor_Skeleton(arm2);

    Gamepad Arm_Gamepad;

    double Kp = 0.05;
    double Ki = 0.0;
    double Kd = 0.0;

    double max_accel = 1500;
    double max_vel = 1500;

    double tolerance = 5;

    double max_output = 0.1;

    public double Reeference = -10;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timer2 = new ElapsedTime();

    boolean toggle1=true;
    boolean toggle2=true;

    public double savedPower = 0;
    boolean firstTime = true;

    double lastPosition;

   public double pow = 1;



    public void init(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor2.init(ahwMap,"Arm2",false,false);

        ArmMotor2.setMaxAccelandVel(Kp,Kd,Ki,max_accel,max_vel);
        ArmMotor2.setPidCoefs(Kp,Kd,Ki);

    }

    public void setPidCoefs(double p, double i, double d) {
        ArmMotor2.setPidCoefs(p,i,d);
    }

    public void update()
    {

        if(Arm_Gamepad.left_bumper) {
            SetPower(-pow);
        }
        else if(Arm_Gamepad.right_bumper) {
            SetPower(pow);
        }
        else {
            SetPower(0);
        }

        if(Arm_Gamepad.dpad_up) {
            if(toggle1) {
                pow+=0.1;
                toggle1=false;
            }
        }
        else toggle1=true;
        if(Arm_Gamepad.dpad_down) {
            if(toggle2) {
                pow-=0.1;
                toggle2=false;
            }
        }
        else toggle2=true;
        /*
        SetPidPower(Reeference);

        if(timer2.seconds()>1) {
            toggle1=true;
        }
        if (Arm_Gamepad.left_trigger>0) {
            if(toggle1) {
                Reeference-=100;
                toggle1=false;
                timer2.reset();
            }
        } else if (Arm_Gamepad.right_trigger>0) {
            if(toggle1) {
                Reeference+=100;
                toggle1=false;
                timer2.reset();
            }
        }
        else {
            toggle1=true;
        }

        lastPosition=ArmMotor2.MotorCurrentPosition();

         */

    }

    public void autoUpdate()
    {
        SetPidPower(Reeference);
        lastPosition = ArmMotor2.MotorCurrentPosition();
    }

    public double getArmPos()
    {
        return ArmMotor2.MotorCurrentPosition();
    }

    public void SetPower(double power) {
        ArmMotor2.SetPower(power);
    }


    public void SetPidPower(double reference)
    {
        if(firstTime) {
            timer.reset();
            firstTime = false;
        }

        //if(!checkSteady())
        //{
            // savedPower = -ArmMotor.returnMpidPower(reference);

            savedPower = ArmMotor2.getPidPower(reference);

            savedPower = addons(savedPower);
        //}


        ArmMotor2.SetPower(savedPower);
    }

    double addons(double pow)
    {

        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;


        return pow;
    }


    public double getReference()
    {
        return Reeference;
    }

    public double getArmPower()
    {
        return ArmMotor2.GetPower();
    }

    public boolean checkSteady() {
        if((abs(Reeference - ArmMotor2.MotorCurrentPosition())<tolerance) && timer.seconds() > 0.3) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setReference(double value)
    {
        Reeference = value;
        firstTime = true;
    }



}
