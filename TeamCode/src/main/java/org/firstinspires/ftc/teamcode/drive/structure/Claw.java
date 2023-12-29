package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public Servo claw1;
    public Servo claw2;

    Gamepad claw_gamepad;

    double open_position1 = 0.65;
    double closed_position1 = 0.75;

    double open_position2 = 0.65;
    double closed_position2 = 0.5;


    public Claw(HardwareMap hwmap, Gamepad claw_gamepad){

        claw1 = hwmap.get(Servo.class, "Claw1");
        claw2 = hwmap.get(Servo.class, "Claw2");

        this.claw_gamepad = claw_gamepad;

        claw1.setPosition(closed_position1);
        claw2.setPosition(closed_position2);
    }

    boolean open = false;
    public boolean toggle = true;

    public void Run()
    {
         if(claw_gamepad.a)
         {
             if(toggle)
             {
                 if (open)
                 {
                     claw1.setPosition(closed_position1);
                     claw2.setPosition(closed_position2);

                 } else {
                     claw1.setPosition(open_position1);
                     claw2.setPosition(open_position2);
                 }

                 open = !open;
             }

             toggle = false;

         }else toggle = true;

    }
}