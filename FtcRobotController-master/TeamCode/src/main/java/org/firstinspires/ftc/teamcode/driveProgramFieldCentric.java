package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Drive Program Field Centric", group = "Main")
public class driveProgramFieldCentric extends LinearOpMode {

    public void runOpMode() {

        //region Variables Setup
        double motorSpeed = 0.8;
        double rawMotorSpeed = 0.8;
        boolean dPMotor = false;
        double leftstickX;
        double leftstickY;


        double turn;

        int armPos = 0;

        boolean homing = false;

        boolean isClosed = false;
        boolean buttonPress = false;

        boolean flipArm = false;

        double waitTime = 0;

        boolean dPSlide = false;
        boolean dPArm = false;
        boolean bumper = false;
        int slidePos = 0;
        int slideTarget = 0;
        double slideTimer = 0;

        int red;
        int blue;


        //endregion

        //region Hardware Map
        TouchSensor limit = hardwareMap.get(TouchSensor.class, "limit");

        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        Servo leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        Servo rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //endregion

        //region Slide Init
        clawServo.setPosition(0.38);

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);

        rightSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);
        rightSlide.setPower(0.25);
        leftSlide.setPower(0.25);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //slide init homing
        while(!limit.isPressed() && (!gamepad2.start && !gamepad1.start) ){
            slideTarget ++;
            rightSlide.setTargetPosition(slideTarget);
            leftSlide.setTargetPosition(slideTarget);
        }

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightSlide.setTargetPosition(10);
        leftSlide.setTargetPosition(10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.5);
        leftSlide.setPower(0.5);
        slideTarget = 25;
        //endregion

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        clawServo.setPosition(0.24);

        while (opModeIsActive()) {


            red = colorSensor.red();
            blue = colorSensor.blue();

            leftstickX = -this.gamepad1.left_stick_x;
            leftstickY = -this.gamepad1.left_stick_y;

            turn = this.gamepad1.right_stick_x;

            //region Motor Speed adjustment
            if (this.gamepad1.right_trigger > 0.1 && rawMotorSpeed < 1) {
                if (!dPMotor) {
                    rawMotorSpeed = rawMotorSpeed + 0.1;
                    dPMotor = true;
                }
            } else if (this.gamepad1.left_trigger > 0.1 && rawMotorSpeed > 0.1) {
                if (!dPMotor) {
                    rawMotorSpeed = rawMotorSpeed - 0.1;
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

            //region Linear Slide Movement
            int rightSlidePos = rightSlide.getCurrentPosition();
            int leftSlidePos = leftSlide.getCurrentPosition();

            if (gamepad2.dpad_up && slidePos < 3 && !dPSlide && slideTimer < System.currentTimeMillis()){
                slidePos++;
                dPSlide = true;
                if (slidePos == 3) {
                    slideTarget = 800;
                }
                else if (slidePos == 2) {
                    slideTarget = 470;
                }
                else if (slidePos == 1) {
                    slideTarget = 150;
                }
                else if (slidePos == 0) {
                    slideTarget = 25;
                }
            }
            else if (gamepad2.dpad_down && slidePos > 0 && !dPSlide && slideTimer < System.currentTimeMillis()){
                slidePos--;
                dPSlide = true;
                if (slidePos == 3) {
                    slideTarget = 800;
                }
                else if (slidePos == 2) {
                    slideTarget = 470;
                }
                else if (slidePos == 1) {
                    slideTarget = 150;
                }
                else if (slidePos == 0) {
                    slideTarget = 25;
                }
            }
            else if (!gamepad2.dpad_down && !gamepad2.dpad_up){
                dPSlide = false;
            }

            if(!homing && gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5) {
                if (gamepad2.right_bumper) {
                    slideTarget += 10;
                }
                else if (gamepad2.left_bumper){
                    slideTarget -= 10;
                }

            }
            //endregion

            //region Slide Homing
            if(gamepad2.start && gamepad2.back && !homing && !limit.isPressed()){
                homing = true;
            }

            if(homing){
                if(limit.isPressed()){
                    homing = false;
                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    rightSlide.setTargetPosition(10);
                    leftSlide.setTargetPosition(10);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else{
                    slideTarget -= 2;
                }
            }
            //endregion

            //region Claw Servo Movement

            if (blue > 200 && !isClosed && waitTime < System.currentTimeMillis()) {
                gamepad1.rumble(250);
                gamepad2.rumble(250);
                isClosed = true;
                clawServo.setPosition(.38);
                slideTimer = System.currentTimeMillis() + 200;
            }
            if (red > 200 && !isClosed && waitTime < System.currentTimeMillis()) {
                gamepad1.rumble(250);
                gamepad2.rumble(250);
                isClosed = true;
                clawServo.setPosition(.38);
                slideTimer = System.currentTimeMillis() + 200;
            }

            if (gamepad2.a){
                if(!buttonPress) {
                    clawServo.setPosition(.24);
                    buttonPress = true;
                    isClosed = false;
                    flipArm = true;
                    waitTime = System.currentTimeMillis() + 100;
                }
            }
            else if (gamepad2.b){
                if(!buttonPress) {
                    clawServo.setPosition(.38);
                    isClosed = true;
                    buttonPress = true;
                    slideTimer = System.currentTimeMillis() + 200;
                }
            }
            else{
                buttonPress = false;
            }

            if(flipArm && System.currentTimeMillis() > waitTime){
                armPos = 0;
                flipArm = false;
            }


            if(gamepad2.dpad_right && armPos < 2){
               if(!dPArm) {
                   armPos++;
                   dPArm = true;
                   clawServo.setPosition(.38);
               }
            }
            if(gamepad2.dpad_left && armPos > 0){
                if(!dPArm) {
                    armPos--;
                    dPArm = true;
                    clawServo.setPosition(.38);
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
            //endregion

            //region Setting Motors
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            leftSlide.setTargetPosition(slideTarget);
            rightSlide.setTargetPosition(slideTarget);
            //endregion

            //region Telemetry Data
            telemetry.addData("Status", "Running");
            telemetry.addData("motorSpeed", motorSpeed);

            telemetry.addData("right slide", rightSlidePos);
            telemetry.addData("left slide", leftSlidePos);

            telemetry.addData("armPos", armPos);

            telemetry.update();
            //endregion
        }
    }
}