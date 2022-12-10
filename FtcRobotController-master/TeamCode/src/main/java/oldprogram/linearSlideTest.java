package oldprogram;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Disabled
@TeleOp(name="Linear Slide Test", group="Concept")
public class linearSlideTest extends OpMode {

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    Servo leftArmServo;
    Servo rightArmServo;
    Servo clawServo;

    int stage = 0;

    @Override
    public void init(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
//
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawServo.setPosition(0.34);

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);

        rightSlide.setTargetPosition(-40);
        leftSlide.setTargetPosition(-40);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);


        //-1000
    }

    @Override
    public void loop(){
        int rightSlidePos = rightSlide.getCurrentPosition();
        int leftSlidePos = leftSlide.getCurrentPosition();
        if (gamepad1.dpad_up){
            rightSlide.setTargetPosition(-1000);
            leftSlide.setTargetPosition(-1000);
        }
        else if (gamepad1.dpad_down){
            rightSlide.setTargetPosition(-40);
            leftSlide.setTargetPosition(-40);
        }

        if (gamepad1.a){
            clawServo.setPosition(.45);
        }
        else if (gamepad1.b){
            clawServo.setPosition(.34);
        }
        if(gamepad1.y){
            leftArmServo.setPosition(0);
            rightArmServo.setPosition(1);
            clawServo.setPosition(.34);
        }
        else if(gamepad1.x){
            leftArmServo.setPosition(.72);
            rightArmServo.setPosition(.3);
            clawServo.setPosition(.34);
        }

       /* if (gamepad1.a && stage == 0) {
            systemTimer.startTimer(4);
            stage++;
        }

        if((systemTimer.check()) && stage == 1) {
            //do something
        }*/
        telemetry.addData("right slide", rightSlidePos);
        telemetry.addData("left slide", leftSlidePos);
        telemetry.update();
    }


}
