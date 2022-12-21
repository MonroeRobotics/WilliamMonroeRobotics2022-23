package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Linear Slide Pos", group="Concept")
public class linearSlidePos extends OpMode {

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    Servo leftArmServo;
    Servo rightArmServo;
    Servo clawServo;
    int stage = 0;

    @Override
    public void init(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");


    }

    @Override
    public void loop(){
        int rightSlidePos = rightSlide.getCurrentPosition();
        int leftSlidePos = leftSlide.getCurrentPosition();

        telemetry.addData("right slide", rightSlidePos);
        telemetry.addData("left slide", leftSlidePos);
        telemetry.update();
    }


}
