/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Auto Program", group = "Main")

public class AutoProgram extends LinearOpMode{

    OpenCvWebcam webcam = null;
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo clawServo;
    Servo leftArmServo;
    Servo rightArmServo;

    double motorSpeed = 1000;

    double leftavgfin;
    double rightavgfin;
    double centeravgfin;

    double direction;
    double magnitude;
    double fRight;
    double bRight;
    double bLeft;
    double fLeft;
    double turn;

    public void homePipe() {
        while (opModeIsActive()) {
            if (centeravgfin < 240){
                motorSpeed = (-1)*centeravgfin + 500;
                direction = -1.57;
                magnitude = 1;
                if(leftavgfin > 20){
                    turn = -.3;
                }
                if(rightavgfin > 20){
                    turn = .3;
                }
            }
            else {
                if(rightavgfin > 0 && leftavgfin > 0){
                    motorSpeed = 200;
                    direction = -1.57;
                    magnitude = -1;
                }
                else if(rightavgfin > 240){
                    motorSpeed = 500;
                    direction = 0;
                    magnitude = 0;
                    turn = .1;
                }
                else if(leftavgfin > 240){
                    motorSpeed = 500;
                    direction = 0;
                    magnitude = 0;
                    turn = -.1;
                }
                else{
                    magnitude = 0;
                    direction = 0;
                    turn = 0;
                    break;
                }

            }


            fRight = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bLeft = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bRight = (motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
            fLeft = (motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));

            //region Setting Motors
            if (bLeft > 0) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
            } else {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
            }
            if (bRight > 0) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
            } else {
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            if (fRight > 0) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
            } else {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
            }
            if (fLeft > 0) {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
            } else {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
            }


            backLeft.setVelocity(Math.abs(bLeft));
            frontRight.setVelocity(Math.abs(fRight));
            backRight.setVelocity(Math.abs(bRight));
            frontLeft.setVelocity(Math.abs(fLeft));
            //endregion


            telemetry.update();
        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    // based on time and power which determines turn direction and turn speed
    public void turnForTime(float time, double turnPower){

        double checkTime =  System.currentTimeMillis();
        double checkTimeEnd = checkTime + time;
        double motorSpeed = 2500;


        // Encoders to set motors to either actively hold position or move freely based on the usage
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        sleep(10);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (turnPower > 0){
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
        }


        while (checkTimeEnd > System.currentTimeMillis() && opModeIsActive()) {
            backLeft.setVelocity(Math.abs(motorSpeed * turnPower));
            frontRight.setVelocity(Math.abs(motorSpeed * turnPower));
            backRight.setVelocity(Math.abs(motorSpeed * turnPower));
            frontLeft.setVelocity(Math.abs(motorSpeed * turnPower));
        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    // Set motors to move, based on time, direction and speed (magnitude)
    public void moveForTime(float time, float deg, double magnitude, double turn){

        double fRight;
        double bRight;
        double bLeft;
        double fLeft;
        double motorSpeed = 2300;

        double direction = deg * (Math.PI / 180);

        double checkTime =  System.currentTimeMillis();
        double checkTimeEnd = checkTime + time;

         // Encoders to set motors to either actively hold position or move freely based on the usage
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(10);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (checkTimeEnd > System.currentTimeMillis() && opModeIsActive()) {

            fRight = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bLeft = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
            bRight = (motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
            fLeft = (motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));


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
        }
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawServo.setPosition(0.29);

        leftArmServo.setPosition(.72);
        rightArmServo.setPosition(.3);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setTargetPosition(-10);
        leftSlide.setTargetPosition(-10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //sets webcam Computer Vision Pipeline to examplePipeline
        webcam.setPipeline(new AutoProgram.pipeDetect());

        //Starts streaming camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        // run until the end of the match (driver presses STOP)

        moveForTime(1200, 270, .5,0);

        sleep(3000);

       moveForTime(1000, 270, .5, -.38);
       rightSlide.setTargetPosition(-1050);
       leftSlide.setTargetPosition(-1050);

        // After park, Move forward, turn, and scan for nearest post
        moveForTime(1450, 270, .5,0 );
//        turnForTime(200,.2);

        sleep(5000);
        homePipe();
        clawServo.setPosition(.45);

        sleep(30000);
    }

    class pipeDetect extends OpenCvPipeline {

        //Initializing Variables
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;

        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Scalar rectColor2 = new Scalar(255.0, 255.0, 0.0);
        Scalar rectColor3 = new Scalar(255.0, 0.0, 255.0);

        //Loops and processes every frame and returns desired changed
        public Mat processFrame(Mat input) {

            //changes Mat input from RGB to HSV and saves to Mat HSV
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            //Creates New rectangle objects for 3 regions, Left, Right, and Center
            Rect leftRect = new Rect(250, 140, 19, 79);
            Rect rightRect = new Rect(370, 140, 19, 79);
            Rect centerRect = new Rect(270, 140, 99, 79);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSV = new Scalar(16,50,50);
            Scalar highHSV = new Scalar(23,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)
            Mat thresh = new Mat();
            Core.inRange(HSV, lowHSV, highHSV, thresh);

            //creates submats(subsections) of Mat HSV using the rectangle regions
            leftCrop = thresh.submat(leftRect);
            rightCrop = thresh.submat(rightRect);
            centerCrop = thresh.submat(centerRect);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar centeravg = Core.mean(centerCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            centeravgfin = centeravg.val[0];

            telemetry.addData("LeftAvg", leftavgfin);
            telemetry.addData("RightAvg", rightavgfin);
            telemetry.addData("CenterAvg", centeravgfin);


            //copies thresh Mat to outPut and draws rectangles regions on the image for user to see
            thresh.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor2, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor3, 2);

            //Returns to display outPut Mat
            return outPut;
        }
    }
}