/*

Code written by the William Monroe Robotics Team
Copyright&copy; 2022 All Rights Reserved*

 */


package oldprogram;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
@Disabled
@TeleOp(name="DriveTest", group="Concept")
public class driveTest extends OpMode {

    //region Declarations
    double rightStick_X;
    double rightStick_Y;
    double leftStick_X;
    double leftStick_Y;

    double magnitude;
    double direction;
    double fRight;
    double fLeft;
    double bLeft;
    double bRight;
    double turn;

    double maxPower;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    //endregion


    @Override
    public void init(){

        //region motorAssignments
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //endregion

    }

    @Override
    public void loop(){
        //region Calculations
        rightStick_X = gamepad1.right_stick_x;
        rightStick_Y = gamepad1.right_stick_y ;
        leftStick_X = gamepad1.left_stick_x;
        leftStick_Y = gamepad1.left_stick_y * -1;
        turn = gamepad1.right_stick_x;

        if (gamepad1.dpad_up){
            frontRight.setPower(.3);
        }
        else if(gamepad1.dpad_left)
        {
            frontLeft.setPower(.3);
        }
        else if (gamepad1.dpad_right){
            backRight.setPower(.3);
        }
        else if(gamepad1.dpad_down){
            backLeft.setPower(.3);
        }
        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        telemetry.update();

        //endregion
    }
}