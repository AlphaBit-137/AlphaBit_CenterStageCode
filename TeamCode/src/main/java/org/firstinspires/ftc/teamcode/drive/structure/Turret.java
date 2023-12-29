package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Turret {

    DcMotorEx turret;

    Motor_Skeleton turret_motor = new Motor_Skeleton(turret);

    Gamepad turret_gamepad;

    Pid_Controller pid_controller;

    double max_angle = 332;

    double Kp = 0.01;
    double Kd;
    double Ki;

    double angle;
    double angle_reference;

    public Turret(HardwareMap hwmap, Gamepad turret_gamepad)
    {
        turret_motor.init(hwmap,"Turret",false,false);

        pid_controller = new Pid_Controller(Kp,Kd,Ki);

        this.turret_gamepad = turret_gamepad;
    }

    public double Ticks_To_Degrees()
    {
        return turret_motor.MotorCurrentPosition() / 1992.6 * 360;
    }

    public void Run() {
        angle = Ticks_To_Degrees();

        if (turret_gamepad.right_trigger != 0 && !cap) {

            angle_reference = angle;
            turret_motor.SetPower(-1);

        } else if(turret_gamepad.left_trigger != 0 && !cap) {

            angle_reference = angle;
            turret_motor.SetPower(1);

        }else SetPidPower();

        if(turret_gamepad.dpad_right)
        {
            angle_reference = angle1;
        }else if(turret_gamepad.dpad_left)
        {
            angle_reference = angle2;
        }

        //angleCap();
    }

    boolean cap = false;

    public void angleCap()
    {
        if(angle > max_angle)
        {
             angle_reference = max_angle;
             cap = true;
        }else cap = false;


    }

    double angle1 = -362;
    double angle2 = 0;

    double max_output = 0.6;

    public void SetPidPower()
    {
        double power = pid_controller.returnPower(angle_reference,angle);

        power = turret_motor.powerConstraints(max_output,power);

        turret_motor.SetPower(power);
    }

    public void SetToReference(double reference)
    {
        angle = Ticks_To_Degrees();
        angle_reference = reference;
        SetPidPower();
    }

}
