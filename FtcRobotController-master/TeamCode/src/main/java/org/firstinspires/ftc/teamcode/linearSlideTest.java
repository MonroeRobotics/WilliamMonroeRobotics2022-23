package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Linear Slide Test", group="Concept")
public class linearSlideTest extends OpMode {

    DcMotor rightSlide;
    DcMotor leftSlide;

    @Override
    public void init(){
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            rightSlide.setPower(0.2);
            leftSlide.setPower(-0.2);
        }
        else if (gamepad1.dpad_down){
            rightSlide.setPower(-0.2);
            leftSlide.setPower(0.2);
        }
        else{
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }
    }
}
