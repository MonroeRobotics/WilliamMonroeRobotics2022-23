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

import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto Program Runner", group = "Main")

public class AutoProgram3 extends LinearOpMode{

    SampleMecanumDrive drive;

    OpenCvWebcam webcam = null;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo clawServo;
    Servo leftArmServo;
    Servo rightArmServo;

    String colorDetectString = "";

    int leftLowBound = 240;
    int leftTarget = 260;
    int rightTarget = 390;
    int rightHighBound = 410;

    boolean isHoming = true;
    double xBounding = 0;
    double yBounding = 0;


    public String detectColor(){
        String detection = colorDetectString;

        return detection;
    }

   /* public void homePipe() {

        webcam.setPipeline(new AutoProgram3.pipeDetect());

        while (opModeIsActive()) {
            motorSpeed = 200;

            if (xBounding > leftLowBound && xBounding < leftTarget && yBounding > rightTarget && yBounding < rightHighBound){
                isHoming = false;
                break;
            }

            if (isHoming) {
                if (xBounding > leftLowBound && yBounding < rightHighBound) {
                    drive.

                }
                else if (xBounding > leftTarget && yBounding > rightTarget) {
                    turn = .2;
                }
                else if (xBounding < leftTarget && yBounding < rightTarget) {
                    turn = -.2;
                }
                else if (xBounding < leftLowBound && yBounding > rightHighBound) {
                    direction = -1.57;
                    magnitude = -1;
                    turn = 0;
                }
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
    }*/

    // based on time and power which determines turn direction and turn speed
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(41, -63.96875, Math.toRadians(270.0)));

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawServo.setPosition(0.24);

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

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(41, -63.96875, Math.toRadians(270.0)))
                .lineToConstantHeading(new Vector2d(37, -59))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToConstantHeading(new Vector2d(33, -37))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(33, -31))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(33, -37))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(315)))
                .build();

        

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //sets webcam Computer Vision Pipeline to examplePipeline
        webcam.setPipeline(new AutoProgram3.colorDetect());

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);

        String color = detectColor();
        telemetry.addData("Color", color);
        telemetry.update();

       rightSlide.setTargetPosition(-1050);
       leftSlide.setTargetPosition(-1050);


        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

//        homePipe(); // Calls method - Locate and position to pipe

//        rightSlide.setTargetPosition(-10);
//        leftSlide.setTargetPosition(-10);
//
//        leftArmServo.setPosition(0);
//        rightArmServo.setPosition(1);

    }

    class pipeDetect extends OpenCvPipeline {

        //Initializing Variables
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat thresh = new Mat();
        Mat dilate = new Mat();
        Mat hierarchy = new Mat();

        Scalar rectColor1 = new Scalar(50, 255, 255);

        //Loops and processes every frame and returns desired changed
        public Mat processFrame(Mat input) {

            //changes Mat input from RGB to HSV and saves to Mat HSV
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            //Creates New rectangle objects for 3 regions, Left, Right, and Center
            Rect leftRect = new Rect(leftTarget, 140, 1, 79);
            Rect rightRect = new Rect(rightTarget, 140, 1, 79);
            Rect leftBound = new Rect(leftLowBound, 140, 1, 79);
            Rect rightBound = new Rect(rightHighBound, 140, 1, 79);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSV = new Scalar(17,70,70);
            Scalar highHSV = new Scalar(25,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

            Core.inRange(HSV, lowHSV, highHSV, thresh);


            Imgproc.morphologyEx(thresh, dilate, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(5, 5)));

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(dilate, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            dilate.copyTo(outPut);


            if (contours.size() != 0) {
                MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];

                for (int i = 0; i < contours.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                }

                double maxVal = 0;
                int maxValIdx = 0;
                for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                    if (maxVal < contourArea) {
                        maxVal = contourArea;
                        maxValIdx = contourIdx;
                    }
                }
                xBounding = boundRect[maxValIdx].x;

                yBounding = boundRect[maxValIdx].x + boundRect[maxValIdx].width;

                telemetry.addData("MaxContor",maxVal);
                Imgproc.rectangle(outPut, boundRect[maxValIdx], new Scalar(150, 80, 100), 3);
            }


            telemetry.addData("x bounding", xBounding);
            telemetry.addData("y bounding", yBounding);


            //copies thresh Mat to outPut and draws rectangles regions on the image for user to see

            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor1, 2);
            Imgproc.rectangle(outPut, leftBound, rectColor1, 2);
            Imgproc.rectangle(outPut, rightBound, rectColor1, 2);


            //Returns to display outPut Mat
            return outPut;
        }
    }


    class colorDetect extends OpenCvPipeline {

        //Initializing Variables
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat threshM = new Mat();
        Mat threshG = new Mat();
        Mat threshA = new Mat();

        Mat dilateM = new Mat();
        Mat dilateG = new Mat();
        Mat dilateA = new Mat();

        Mat cropM = new Mat();
        Mat cropG = new Mat();
        Mat cropA = new Mat();

        Scalar rectColor1 = new Scalar(50, 255, 255);

        //Loops and processes every frame and returns desired changed
        public Mat processFrame(Mat input) {

            //changes Mat input from RGB to HSV and saves to Mat HSV
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            input.copyTo(outPut);

            //Creates New rectangle objects for 3 regions, Left, Right, and Center
            Rect centerScreen = new Rect(270, 150, 100, 60);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSVM = new Scalar(156,40,40);
            Scalar highHSVM = new Scalar(169,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

            Core.inRange(HSV, lowHSVM, highHSVM, threshM);

            Scalar lowHSVG = new Scalar(81,40,40);
            Scalar highHSVG = new Scalar(93,255,255);


            Core.inRange(HSV, lowHSVG, highHSVG, threshG);

            Scalar lowHSVA = new Scalar(90,40,40);
            Scalar highHSVA = new Scalar(99,255,255);


            Core.inRange(HSV, lowHSVA, highHSVA, threshA);

            cropM = threshM.submat(centerScreen);
            cropG = threshG.submat(centerScreen);
            cropA = threshA.submat(centerScreen);

            Scalar avgValueM = Core.mean(cropM);
            Scalar avgValueG = Core.mean(cropG);
            Scalar avgValueA = Core.mean(cropA);

            if (avgValueM.val[0] > avgValueG.val[0] && avgValueM.val[0] > avgValueA.val[0]){
                threshM.copyTo(outPut);
                colorDetectString = "M";

            }

            else if (avgValueG.val[0] > avgValueM.val[0] && avgValueG.val[0] > avgValueA.val[0]){
                threshG.copyTo(outPut);
                colorDetectString = "G";
            }

            else{
                threshA.copyTo(outPut);
                colorDetectString = "A";
            }

            Imgproc.rectangle(outPut, centerScreen, rectColor1, 1);

            //Returns to display outPut Mat
            return outPut;
        }
    }


}