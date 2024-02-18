package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public Servo claw1;
    public Servo claw2;
    public Servo centration;

    Gamepad claw_gamepad;

    double open_position1 = 0.425;
    double closed_position1 = 0.479;

    double open_position2 = 0.4;
    double closed_position2 = 0.31;

    double down_pos = 0.225;
    double init_pos = 0.825;
    double up_pos = 0.7;



    public Claw(HardwareMap hwmap, Gamepad claw_gamepad){

        claw1 = hwmap.get(Servo.class, "Claw1");
        claw2 = hwmap.get(Servo.class, "Claw2");
        centration = hwmap.get(Servo.class,"Centration");

        this.claw_gamepad = claw_gamepad;

        claw1.setPosition(closed_position1);
        claw2.setPosition(closed_position2);
        centration.setPosition(init_pos);

    }

    public boolean open = false;
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

         if(claw_gamepad.dpad_right) {
             claw1.setPosition(closed_position1);
             claw2.setPosition(closed_position2);
             centration.setPosition(init_pos);
         }
         else if(claw_gamepad.dpad_left) {
             claw1.setPosition(open_position1);
             claw2.setPosition(open_position2);
             centration.setPosition(down_pos);
         }
         else if(claw_gamepad.dpad_up) {
             centration.setPosition(up_pos);
         }

    }
}
