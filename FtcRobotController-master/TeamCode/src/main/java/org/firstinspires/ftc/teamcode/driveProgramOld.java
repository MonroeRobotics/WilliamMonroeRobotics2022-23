package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive Old", group = "Main")
// Wheel Movement
public class driveProgramOld extends LinearOpMode {

    public void runOpMode() {
        //region Variables Setup
        double motorSpeed = 2000;
        boolean dP = false;
        double leftstickX;
        double leftstickY;
        double direction;
        double magnitude;
        double fRight;
        double bRight;
        double bLeft;
        double fLeft;
        double turn;





        //region Hardware Map
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //endregion

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {


            //region Motor Speed adjustment
            if (this.gamepad1.dpad_up && motorSpeed < 10000) {
                if (!dP) {
                    motorSpeed = motorSpeed + 1000;
                    dP = true;
                }
            } else if (this.gamepad1.dpad_down && motorSpeed > 1000) {
                if (dP == false) {
                    motorSpeed = motorSpeed - 1000;
                    dP = true;
                }
            } else {
                dP = false;
            }

            //endregion

            leftstickX = this.gamepad1.left_stick_x;
            leftstickY = -this.gamepad1.left_stick_y;

            turn = -this.gamepad1.right_stick_x;
            //region Math For Wheel Movement
            direction = Math.atan2(leftstickY, leftstickX);
            magnitude = Math.sqrt(Math.pow(leftstickX, 2) + Math.pow(leftstickY, 2)) * 1.5;

            fRight = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bLeft = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bRight = (motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
            fLeft = (motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));

            /*
            if (fRight > 1 || fRight < -1){
                fLeft = (fLeft / Math.abs(fRight));
                fRight = (fRight / Math.abs(fRight));
                bRight = (bRight / Math.abs(fRight));
                bLeft = (bLeft / Math.abs(fRight));
            }

            if (fLeft > 1 || fLeft < -1){
                fLeft = (fLeft / Math.abs(fLeft));
                fRight = (fRight / Math.abs(fLeft));
                bLeft = (bLeft / Math.abs(fLeft));
                bRight = (bRight / Math.abs(fLeft));
            }
            */
            //endregion

            //region Setting Motors
            if (bLeft > 0){
                backLeft.setDirection(DcMotor.Direction.FORWARD);
            }
            else {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
            }
            if (bRight > 0){
                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else {
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            if (fRight > 0){
                frontRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
            }
            if (fLeft > 0){
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
            }


            backLeft.setVelocity(Math.abs(bLeft));
            frontRight.setVelocity(Math.abs(fRight));
            backRight.setVelocity(Math.abs(bRight));
            frontLeft.setVelocity(Math.abs(fLeft));
            //endregion

            //region Telemetry Data
            telemetry.addData("motorSpeed", motorSpeed);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            //endregion
        }
    }
}
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//$ claim $5,000,000,000
//add code here
//add code here
//add code here
//add code here
//add code here
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod
//add co
//add c
//add
//ad
//a
//add code here
//add code her
//add code he
//add code h
//add code
//add cod