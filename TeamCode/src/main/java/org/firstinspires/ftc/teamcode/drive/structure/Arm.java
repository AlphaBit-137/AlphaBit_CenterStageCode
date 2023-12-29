package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Arm {

    public DcMotorEx arm1;

    public Motor_Skeleton ArmMotor1 = new Motor_Skeleton(arm1);

    Gamepad Arm_Gamepad;

    double ku=0.05;
    double Kp = 0.007;
    double Ki = 0;
    double Kd = 0.001;

    double max_accel = 1500;
    double max_vel = 1500;

    double max_output = 0.2;

    public double pidPower=0;

    public double Reeference = 0;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timer2 = new ElapsedTime();

    public double savedPower = 0;
    boolean firstTime = true;

    double pow=0.2;

    double lastPosition;
    boolean toggle1=true;

    double max_pos = 1000;
    double min_pos = 100;
    double up_pos = 172;
    double down_pos = 0;
    static double encoder_position;

    boolean gpad=false;

    public void init(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor1.init(ahwMap,"Arm",false,true);

        ArmMotor1.setMaxAccelandVel(Kp,Kd,Ki,max_accel,max_vel);
        ArmMotor1.setPidCoefs(Kp,Kd,Ki);
    }

    public void SetPidCoefs(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public void update()
    {
        /*if(Arm_Gamepad.left_bumper) {
            SetPower(pow);
        }
        else if(Arm_Gamepad.right_bumper) {
            SetPower(-pow);
        }
        else {
            SetPower(0);
        }
        SetPidPower(Reeference);

            if(timer2.seconds()>1) {
                toggle1=true;
            }
            if (Arm_Gamepad.left_bumper) {
                if(toggle1) {
                    Reeference-=100;
                    toggle1=false;
                    timer2.reset();
                }
            } else if (Arm_Gamepad.right_bumper) {
                if(toggle1) {
                    Reeference+=100;
                    toggle1=false;
                    timer2.reset();
                }
            }
            else {
                toggle1=true;
            }
            */
        encoder_position = ArmMotor1.MotorCurrentPosition();

        if(Arm_Gamepad.right_trigger>0 /*&& encoder_position < max_pos*/)
        {
            ArmMotor1.SetPower(pow);
            Reeference = encoder_position;
        }
        else if(Arm_Gamepad.left_trigger>0 /*&& encoder_position > min_pos*/)
        {
            ArmMotor1.SetPower(-pow);
            Reeference = encoder_position;
        }
        else SetPidPower(Reeference);

        if(Arm_Gamepad.dpad_up)
        {
            Reeference = 162;
        }
        else if(Arm_Gamepad.dpad_down)
        {
            Reeference = down_pos;
        }

        //}
        /*else {

            SetPidPower(Reeference);

            if (Arm_Gamepad.dpad_up) {
                setReference(2100);
            }


            if (Arm_Gamepad.dpad_down) {
                setReference(-2);
            }*/

            //lastPosition = ArmMotor1.MotorCurrentPosition();


    }

    public void tuneUpdate(boolean gpad) {
        SetPidPower(Reeference);
        this.gpad=gpad;
    }

    public void autoUpdate()
    {
        SetPidPower(Reeference);
        lastPosition = ArmMotor1.MotorCurrentPosition();
    }

    public double getArmPos()
    {
        return ArmMotor1.MotorCurrentPosition();
    }

    public void SetPower(double power) {
        ArmMotor1.SetPower(power);
    }


    /*public void SetPidPower(double reference)
    {
        if(firstTime) {
            timer.reset();
            firstTime = false;
        }

        if(!checkSteady())
        {
            // savedPower = -ArmMotor.returnMpidPower(reference);

            savedPower = ArmMotor1.getPidPower(reference);

            savedPower = addons(savedPower);
        }


        ArmMotor1.SetPower(savedPower);
    }*/

    /*public void SetPidPower(double reference)
    {
        double pow = ArmMotor1.getPidPower(reference);

        pow = addons(pow);

        pidPower = pow;

        if(gpad) {
            ArmMotor1.SetPower(pow);
        }
        else {
            ArmMotor1.SetPower(0);
        }
    }*/

    public void SetPidPower(double reference)
    {
        double pow = ArmMotor1.getPidPower(reference);

        pow = addons(pow);

        ArmMotor1.SetPower(pow);
    }

    public double getCalculatedPow() {
        return pidPower;
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
        return ArmMotor1.GetPower();
    }

    public boolean checkSteady() {
        if((lastPosition == ArmMotor1.MotorCurrentPosition()) && timer.seconds() > 0.3) {
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
