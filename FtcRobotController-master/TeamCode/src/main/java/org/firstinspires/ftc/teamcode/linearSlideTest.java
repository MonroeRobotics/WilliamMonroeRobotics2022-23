package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Timer;

@TeleOp(name="Linear Slide Test", group="Concept")
public class linearSlideTest extends OpMode {

    Timer countdown;
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    systemTimer systemTimer;

    @Override
    public void init(){
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setVelocity(1000);
            leftSlide.setVelocity(1000);
        }
        else if (gamepad1.dpad_down){
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightSlide.setVelocity(1000);
            leftSlide.setVelocity(1000);
        }
        else{
            rightSlide.setVelocity(0);
            leftSlide.setVelocity(0);
        }
        systemTimer.startTimer(4);
    }
}
