
package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="set servo 0.5", group="Linear OpMode")
//@Disabled
public class setServo0point5 extends LinearOpMode {

    ServoImplEx h, t1, t2, s,flower;


    @Override
    public void runOpMode() {

        t1     = hardwareMap.get(ServoImplEx.class,"turret1");
        t2     = hardwareMap.get(ServoImplEx.class,"turret2");
        flower = hardwareMap.get(ServoImplEx.class,"flower");
        //h = hardwareMap.get(Servo.class,"hood");
        //s = hardwareMap.get(Servo.class,"swingArm");
        //h.setDirection(Servo.Direction.REVERSE);
        t1.setDirection(ServoImplEx.Direction.REVERSE);
        t2.setDirection(ServoImplEx.Direction.REVERSE);




        waitForStart();

        while (opModeIsActive()) {
        t1.setPwmRange(new PwmControl.PwmRange(550, 2450));
        t2.setPwmRange(new PwmControl.PwmRange(550, 2450));
        flower.setPosition(.5);
        t1.setPosition(0.5);
        t2.setPosition(0.5);
        //h.setPosition(0.5);
        //s.setPosition(0.55);
        //telemetry.addData("hood position", h.getPosition());
        telemetry.update();
        }
    }
}