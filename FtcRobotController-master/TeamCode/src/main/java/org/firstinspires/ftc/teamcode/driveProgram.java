package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive Program", group = "Main")
public class driveProgram extends LinearOpMode {

    public void runOpMode() {
        //region Variables Setup
        double motorSpeed = 2000;
        boolean dPMotor = false;
        double leftstickX;
        double leftstickY;

        double direction;
        double magnitude;
        double fRight;
        double bRight;
        double bLeft;
        double fLeft;
        double turn;

        boolean dPSlide = false;
        int slidePos = 0;
        //endregion

        //region Hardware Map
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        Servo leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        Servo rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //endregion

        //region Slide Init
        clawServo.setPosition(0.29);

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);

        rightSlide.setTargetPosition(-10);
        leftSlide.setTargetPosition(-10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);

        //endregion

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            leftstickX = this.gamepad1.left_stick_x;
            leftstickY = -this.gamepad1.left_stick_y;

            turn = this.gamepad1.right_stick_x;

            //region Motor Speed adjustment
            if (this.gamepad1.dpad_up && motorSpeed < 10000) {
                if (!dPMotor) {
                    motorSpeed = motorSpeed + 1000;
                    dPMotor = true;
                }
            } else if (this.gamepad1.dpad_down && motorSpeed > 1000) {
                if (!dPMotor) {
                    motorSpeed = motorSpeed - 1000;
                    dPMotor = true;
                }
            } else {
                dPMotor = false;
            }

            //endregion

            //region Math For Wheel Movement
            direction = Math.atan2(leftstickY, leftstickX);
            magnitude = Math.sqrt(Math.pow(leftstickX, 2) + Math.pow(leftstickY, 2)) * 1.5;

            fRight = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bLeft = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bRight = (motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
            fLeft = (motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));

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

            //region Linear Slide Movement
            int rightSlidePos = rightSlide.getCurrentPosition();
            int leftSlidePos = leftSlide.getCurrentPosition();

            if (gamepad2.dpad_up && slidePos < 3 && !dPSlide){
                slidePos++;
                dPSlide = true;
            }
            else if (gamepad2.dpad_down && slidePos > 0 && !dPSlide){
                slidePos--;
                dPSlide = true;
            }
            else if (!gamepad2.dpad_down && !gamepad2.dpad_up){
                dPSlide = false;
            }

            if (slidePos ==  3){
                rightSlide.setTargetPosition(-1050);
                leftSlide.setTargetPosition(-1050);
            }
            else if (slidePos ==  2){
                rightSlide.setTargetPosition(-480);
                leftSlide.setTargetPosition(-480);
            }
            else if (slidePos ==  1){
                rightSlide.setTargetPosition(-160);
                leftSlide.setTargetPosition(-160);
            }
            else if (slidePos ==  0){
                rightSlide.setTargetPosition(-10);
                leftSlide.setTargetPosition(-10);
            }
            //endregion

            //region Claw Servo Movement
            if (gamepad2.a){
                clawServo.setPosition(.45);
            }
            else if (gamepad2.b){
                clawServo.setPosition(.29);
            }
            if(gamepad2.y){
                leftArmServo.setPosition(0);
                rightArmServo.setPosition(1);
                clawServo.setPosition(.28);
            }
            else if(gamepad2.x){
                leftArmServo.setPosition(.72);
                rightArmServo.setPosition(.3);
                clawServo.setPosition(.28);
            }
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
            telemetry.addData("direcetion", direction);
            telemetry.addData("mag", magnitude);
            telemetry.addData("Status", "Initialized");

            telemetry.addData("right slide", rightSlidePos);
            telemetry.addData("left slide", leftSlidePos);
            telemetry.addData("Slide Position", slidePos);

            telemetry.update();
            //endregion
        }
    }
}