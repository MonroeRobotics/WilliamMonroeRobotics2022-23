package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive Program", group = "Main")
public class driveProgram extends LinearOpMode {

    public void runOpMode() {
        //region Variables Setup
        double motorSpeed = 2000;
        double rawMotorSpeed = 2000;
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

        int armPos = 0;

        boolean isClosed = false;

        boolean dPSlide = false;
        boolean dPArm = false;
        boolean bumper = false;
        int slidePos = 0;

        int red;
        int green;
        int blue;

        double distance;


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

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu.initialize(parameters);

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
        clawServo.setPosition(0.24);

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);

        rightSlide.setTargetPosition(-10);
        leftSlide.setTargetPosition(-10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);

        //endregion

        //Wait for Calibrate Gyro
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        clawServo.setPosition(0.38);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            distance = distanceSensor.getDistance(DistanceUnit.MM);

            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();

            leftstickX = this.gamepad1.left_stick_x;
            leftstickY = -this.gamepad1.left_stick_y;

            turn = this.gamepad1.right_stick_x;

            //region Motor Speed adjustment
            if (this.gamepad1.right_trigger > 10 && rawMotorSpeed < 10000) {
                if (!dPMotor) {
                    rawMotorSpeed = rawMotorSpeed + 500;
                    dPMotor = true;
                }
            } else if (this.gamepad1.left_trigger > 10 && rawMotorSpeed > 1000) {
                if (!dPMotor) {
                    rawMotorSpeed = rawMotorSpeed - 500;
                    dPMotor = true;
                }
            } else {
                dPMotor = false;
            }

            if(gamepad1.right_bumper && !bumper){
                motorSpeed = motorSpeed * 1.5;
                bumper = true;
            }
            else if(gamepad1.left_bumper && !bumper){
                motorSpeed = motorSpeed / 4;
                bumper = true;
            }
            else if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                motorSpeed = rawMotorSpeed;
                bumper = false;
            }

            //endregion

            //region Math For Wheel Movement
            direction = Math.atan2(leftstickY, leftstickX);
            magnitude = Math.sqrt(Math.pow(leftstickX, 2) + Math.pow(leftstickY, 2)) * 1.5;


            //region precision movement
            if(gamepad1.dpad_up){
                magnitude = .1;
                direction = Math.toRadians(90.0);
            }
            else if(gamepad1.dpad_down){
                magnitude = .1;
                direction = Math.toRadians(270);
            }
            else if(gamepad1.dpad_left){
                magnitude = .1;
                direction = Math.toRadians(180);
            }
            else if(gamepad1.dpad_right){
                magnitude = .1;
                direction = Math.toRadians(0);
            }

                //endregion

            fRight = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bLeft = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bRight = -(motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
            fLeft = -(motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
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
                rightSlide.setTargetPosition(-530);
                leftSlide.setTargetPosition(-530);
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

            if ((red > 400 && distance < 40) && !isClosed) {
                gamepad1.rumble(250);
                gamepad2.rumble(250);
                isClosed = true;
                clawServo.setPosition(.24);
            }

            if (gamepad2.a){
                clawServo.setPosition(.38);
                isClosed = false;
            }
            else if (gamepad2.b){
                clawServo.setPosition(.24);
                isClosed = true;
            }


            if(gamepad2.dpad_right && armPos < 3){
               if(!dPArm) {
                   armPos++;
                   dPArm = true;
                   clawServo.setPosition(.20);
               }
            }
            if(gamepad2.dpad_left && armPos > 0){
                if(!dPArm) {
                    armPos--;
                    dPArm = true;
                    clawServo.setPosition(.20);
                }
            }
            else if (!gamepad2.dpad_left && !gamepad2.dpad_right){
                dPArm = false;
            }

            if(armPos == 0){
                leftArmServo.setPosition(0);
                rightArmServo.setPosition(1);
            }
            else if(armPos == 1){
                leftArmServo.setPosition(.3);
                rightArmServo.setPosition(.75);
            }
            else if(armPos == 2){
                leftArmServo.setPosition(.75);
                rightArmServo.setPosition(.3);
            }
            else if(armPos == 3){
                leftArmServo.setPosition(1);
                rightArmServo.setPosition(0);
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
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
            }
            else {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
            }


            backLeft.setVelocity(Math.abs(bLeft));
            frontRight.setVelocity(Math.abs(fRight));
            backRight.setVelocity(Math.abs(bRight));
            frontLeft.setVelocity(Math.abs(fLeft));
            //endregion

            //region Telemetry Data
            telemetry.addData("Status", "Running");
            telemetry.addData("motorSpeed", motorSpeed);
            telemetry.addData("direction", direction);
            telemetry.addData("mag", magnitude);

            telemetry.addData("motor Ticks", frontLeft.getCurrentPosition());

            telemetry.addData("right slide", rightSlidePos);
            telemetry.addData("left slide", leftSlidePos);

            telemetry.addData("Distance", distance);

            telemetry.addData("armPos", armPos);

            telemetry.update();
            //endregion
        }
    }
}