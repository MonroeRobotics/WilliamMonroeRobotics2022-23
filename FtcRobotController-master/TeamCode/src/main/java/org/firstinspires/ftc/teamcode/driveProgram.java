/*

Code written by the William Monroe Robotics Team
Copyright&copy; 2022All Rights Reserved*

 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drive Program")
public class driveProgram extends OpMode {

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
        rightStick_Y = gamepad1.right_stick_y;
        leftStick_X = gamepad1.left_stick_x;
        leftStick_Y = gamepad1.left_stick_y;
        turn = rightStick_X;

        magnitude = Math.sqrt(Math.pow(leftStick_X, 2) + Math.pow(leftStick_Y, 2));

        direction = Math.atan2(leftStick_Y, leftStick_X);
        fRight = Math.sin(direction-1.0/4.0*Math.PI)*magnitude + turn;
        bLeft = -Math.sin(direction-1.0/4.0*Math.PI)*magnitude + turn;
        fLeft = Math.sin(direction+1.0/4.0*Math.PI)*magnitude + turn;
        bRight = -Math.sin(direction+1.0/4.0*Math.PI)*magnitude + turn;

        //endregion
        //region setPower
        frontLeft.setPower(fLeft);
        frontRight.setPower(fRight);
        backLeft.setPower(bLeft);
        backRight.setPower(bRight);
    //endregion
    }
}
