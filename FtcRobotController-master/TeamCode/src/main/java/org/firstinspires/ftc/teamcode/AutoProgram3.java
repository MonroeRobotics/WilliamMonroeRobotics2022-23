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

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name = "Auto Program 3", group = "Main")

public class AutoProgram3 extends OpMode {

    SampleMecanumDrive drive;

    Trajectory traj;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;

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

    boolean isHoming = false;
    boolean homed = false;
    double xBounding = 0;
    double yBounding = 0;

    int stage = 0;


    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,   // Then, we follow another lineTo() trajectory
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    public String detectColor(){
        String detection = colorDetectString;

        return detection;
    }

    public void homePipe() {
            if (xBounding > leftLowBound && xBounding < leftTarget && yBounding > rightTarget && yBounding < rightHighBound){
                isHoming = false;
                drive.setDrivePower(new Pose2d(0, 0, 0));
                clawServo.setPosition(0.38);
            }

            if (isHoming) {
                if (xBounding > leftLowBound && yBounding < rightHighBound) {
                    drive.setDrivePower(new Pose2d(-.1, 0, 0));
                    telemetry.addData("Line","0");
                }
                else if (xBounding > leftTarget && yBounding > rightTarget) {
//                    drive.turnAsync(drive.getExternalHeading() - 0.02);
                    drive.setDrivePower(new Pose2d(0, .1, 0));
                    telemetry.addData("Line","1");
                }
                else if (xBounding < leftTarget && yBounding < rightTarget) {
//                    drive.turnAsync(drive.getExternalHeading() + 0.02);
                    drive.setDrivePower(new Pose2d(0, -.1, 0));
                    telemetry.addData("Line","2");
                }
                else if (xBounding < leftLowBound && yBounding > rightHighBound) {
                   drive.setDrivePower(new Pose2d(0.1, 0, 0));
                    telemetry.addData("Line","3");
                }
            }
            //endregion
            telemetry.update();
    }

    // based on time and power which determines turn direction and turn speed
    @Override
    public void init() {

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

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //sets webcam Computer Vision Pipeline to examplePipeline
        webcam.setPipeline(new AutoProgram3.colorDetect());

        //Starts streaming camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

//        sleep(500);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(41, -63.96875, Math.toRadians(270.0)));

        traj = drive.trajectoryBuilder(new Pose2d(41, -63.96875, Math.toRadians(270.0)))
                .lineToConstantHeading(new Vector2d(37, -59))
                .build();

        traj2 = drive.trajectoryBuilder(traj.end())
                .lineToConstantHeading(new Vector2d(33, -37))
                .addDisplacementMarker(() -> {
                    colorDetectString = detectColor();
                    telemetry.addData("color", colorDetectString);
                    telemetry.update();
                })
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(33, -31))
                .addDisplacementMarker(() -> {
                    rightSlide.setTargetPosition(-1050);
                    leftSlide.setTargetPosition(-1050);
                })
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(33, -37))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(8, -35, Math.toRadians(300)))
                .addDisplacementMarker(() -> {
                    webcam.setPipeline(new AutoProgram3.pipeDetect());
                    isHoming = true;
                })
                .build();

        currentState = State.TRAJECTORY_1;

        drive.followTrajectoryAsync(traj);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    public void loop(){
        if(!isHoming) {
            drive.update();
        }
        colorDetectString = detectColor();

        switch (currentState) {
            case TRAJECTORY_1:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_2;
                    drive.followTrajectoryAsync(traj2);
                }
                break;
            case TRAJECTORY_2:
                // Check if the drive class is busy following the trajectory
                // Move on to the next state, TURN_1, once finished
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_3;
                    drive.followTrajectoryAsync(traj3);
                }
                break;
            case TRAJECTORY_3:
                // Check if the drive class is busy turning
                // If not, move onto the next state, TRAJECTORY_3, once finished
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_4;
                    drive.followTrajectoryAsync(traj4);
                }
                break;
            case TRAJECTORY_4:
                // Check if the drive class is busy following the trajectory
                // If not, move onto the next state, WAIT_1
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_5;
                    drive.followTrajectoryAsync(traj5);
                }
                break;
            case TRAJECTORY_5:
                // Check if the timer has exceeded the specified wait time
                // If so, move on to the TURN_2 state
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                // Do nothing in IDLE
                // currentState does not change once in IDLE
                // This concludes the autonomous program
                break;
        }
        if(isHoming) {
            homePipe(); // Calls method - Locate and position to pipe
        }

        if(homed){
            clawServo.setPosition(.3);
        }
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