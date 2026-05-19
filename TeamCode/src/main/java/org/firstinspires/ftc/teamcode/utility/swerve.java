package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;

@TeleOp(name="swerve test", group="swerve")
//@Disabled
public class swerve extends LinearOpMode {
    public hardwareSubNewBot h;
    public varSub var;
    Servo r;
    DcMotorEx m1;



    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        var = new varSub();


        r = hardwareMap.get(Servo.class, "hood");
        m1 = hardwareMap.get(DcMotorEx.class, "intake");



        waitForStart();

        while (opModeIsActive()) {

            r.setPosition(gamepad1.right_stick_x);
            m1.setPower(gamepad1.left_stick_x);



        }
    }


}