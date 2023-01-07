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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

@Autonomous(name = "Auto Program 5 + 1 Epic Auto", group = "Main")

public class AutoProgram5Cones extends OpMode {

    SampleMecanumDrive drive;

    DistanceSensor distanceSensor;

    Trajectory traj;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj3_2;
    Trajectory traj4;
    Trajectory traj5;

    Trajectory centerTraj;

    TrajectorySequence toPoll;
    TrajectorySequence toCone;

    TrajectorySequence toPollCenter;
    TrajectorySequence toConeCenter;


    TrajectorySequence park2;
    TrajectorySequence park3;

    Pose2d pipePose;


    Pose2d conePose;

    OpenCvWebcam webcam = null;

    OpenCvWebcam webcamfront = null;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo clawServo;
    Servo leftArmServo;
    Servo rightArmServo;

    double waitTime;
    double currentTime;

    WebcamName webcamNameFront;

    int cameraMonitorViewId;

    String colorDetectString = "";
    String colorDetected = "";

    int coneCount = 0;

    double lBounding;
    double rBounding;

    int leftTarget = 330;
    int rightTarget = 440;

    double baseSpeedVert = 0.01;
    double baseSpeedHorz = 0.01;
    double motorVertical;
    double motorHorizontal;
    double multiplier = 0.25;
    int aOffset = 100;

    int leftLowBoundCone = 200;
    int leftTargetCone = 220;
    int rightTargetCone = 430;
    int rightHighBoundCone = 450;

    boolean isHoming = false;
    boolean isConeHoming = false;
    boolean homed = false;
    double conexBounding = 0;
    double coneyBounding = 0;


    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_3_2,
        TRAJECTORY_4,
        TRAJECTORY_5,
        TO_POLL,
        CONE_TO_CENTER,
        POLL_TO_CENTER,
        TO_CONE,
        CENTER,
        PARK,
        IDLE,
        HOMING
    }

    State currentState;

    public String detectColor(){
        String detection = colorDetectString;

        return detection;
    }

    public void homeCone() {
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
            if (distanceSensor.getDistance(DistanceUnit.MM) <= 38){
                clawServo.setPosition(0.24);
                isConeHoming = false;
                drive.setDrivePower(new Pose2d(0, 0, 0));
                drive.update();
                conePose = drive.getPoseEstimate();

                currentState = State.CONE_TO_CENTER;
            }

            else if (isConeHoming) {
                if (conexBounding > leftTargetCone && coneyBounding > rightTargetCone) {
//                    drive.turnAsync(drive.getExternalHeading() - 0.02);
                    drive.setDrivePower(new Pose2d(0, -.1, 0));
                }
                else if (conexBounding < leftTargetCone && coneyBounding < rightTargetCone) {
//                    drive.turnAsync(drive.getExternalHeading() + 0.02);
                    drive.setDrivePower(new Pose2d(0, .1, 0));
                }
                else if (distanceSensor.getDistance(DistanceUnit.MM) >= 38) {
                    drive.setDrivePower(new Pose2d(.2, 0, 0));
                }
//                else if (xBounding < leftLowBound && yBounding > rightHighBound) {
//                   drive.setDrivePower(new Pose2d(0.1, 0, 0));
//                }
            }
            //endregion
            telemetry.update();
    }
    public void homePipe() {

        //find width of target and poles
        double poleWidth = rBounding - lBounding;
        double targetWidth = rightTarget - leftTarget;

        //find center of poles and target
        double poleCenter = lBounding + (poleWidth / 2);
        double targetCenter = leftTarget + (targetWidth / 2);

        //get center distance
        double cOffset = targetCenter - poleCenter;

        double wOffset = targetWidth - poleWidth;



        //get motor powers

        if (wOffset < 0){
            baseSpeedVert = Math.abs(baseSpeedVert) * -1;
        }
        else{
            baseSpeedVert = Math.abs(baseSpeedVert);
        }
        if (cOffset < 0){

            baseSpeedHorz = Math.abs(baseSpeedHorz) * -1;
        }
        else{
            baseSpeedHorz = Math.abs(baseSpeedHorz);
        }

        if(Math.abs(wOffset) > 15) {
            motorVertical = multiplier * (wOffset / aOffset) + baseSpeedVert;
        }
        else{
            motorVertical = 0;
        }

        if(Math.abs(cOffset) > 15) {
            motorHorizontal = multiplier * (cOffset / aOffset) + baseSpeedHorz;
        }
        else{
            motorHorizontal = 0;
        }

        if (Math.abs(wOffset) < 5 && Math.abs(cOffset) < 5){
            isHoming = false;

            drive.setDrivePower(new Pose2d(0, 0, 0));
            drive.update();
            pipePose = drive.getPoseEstimate();


            clawServo.setPosition(0.38);
            homed = true;
            currentTime = System.currentTimeMillis();
            waitTime = System.currentTimeMillis() + 500.0;
            currentState = State.TRAJECTORY_4;
            return;
        }

        //set drive motors
        drive.setDrivePower(new Pose2d(-motorVertical, -motorHorizontal, 0));

        telemetry.addData("cOff", cOffset);
        telemetry.addData("wOff", wOffset);
        telemetry.addData("motorHorz", motorHorizontal);
        telemetry.addData("motorVerti", motorVertical);
        telemetry.addData("baseHorz", baseSpeedHorz);
        telemetry.addData("baseVert", baseSpeedVert);

//        if (Math.abs(wOffset) < 5 && Math.abs(cOffset) < 5){
//            stop();
//        }
    }


    // based on time and power which determines turn direction and turn speed
    @Override
    public void init() {

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");


        clawServo.setPosition(0.24);

        leftArmServo.setPosition(.72);
        rightArmServo.setPosition(.3);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setTargetPosition(10);
        leftSlide.setTargetPosition(10);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(0.4);
        leftSlide.setPower(0.4);




        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewportContainerIds[0]);



        //Starts streaming camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //sets webcam Computer Vision Pipeline to examplePipeline
                webcam.setPipeline(new AutoProgram5Cones.colorDetect());
//        webcam.setPipeline(new AutoProgram4.coneDetect());
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        webcamNameFront = hardwareMap.get(WebcamName.class, "webcamfront");

        webcamfront = OpenCvCameraFactory.getInstance().createWebcam(webcamNameFront, viewportContainerIds[1]);
        webcamfront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcamfront.setPipeline(new AutoProgram5Cones.coneDetect());
                webcamfront.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        //sets webcam Computer Vision Pipeline to examplePipeline

//        webcam.setPipeline(new AutoProgram4.coneDetect());

        //Starts streaming camera


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(41, -63.96875, Math.toRadians(270.0)));

        traj = drive.trajectoryBuilder(new Pose2d(41, -63.96875, Math.toRadians(270.0)))
                .lineToConstantHeading(new Vector2d(36, -59))
                .addDisplacementMarker(() -> {
                    colorDetected = detectColor();
                    telemetry.addData("color", colorDetectString);
                    telemetry.update();
                })
                .build();

        traj2 = drive.trajectoryBuilder(traj.end())
                .lineToConstantHeading(new Vector2d(36, 0))
                .addDisplacementMarker(() -> {
                    rightSlide.setTargetPosition(750);
                    leftSlide.setTargetPosition(750);
                })
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .addDisplacementMarker(() -> {
                    webcam.setPipeline(new AutoProgram5Cones.pipeDetect());
                })
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(300)))
                .build();

        traj3_2 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(36, -11.8))
                .addDisplacementMarker(() -> {
                    isHoming = true;
                })
                .build();
        currentState = State.TRAJECTORY_1;

        drive.followTrajectoryAsync(traj);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop(){

        switch (currentState) {
            case TRAJECTORY_1:
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_2;
                    drive.followTrajectoryAsync(traj2);
                }
                break;
            case TRAJECTORY_2:
                if (!drive.isBusy()) {
                    currentState = State.TRAJECTORY_3;
                    drive.followTrajectoryAsync(traj3);
                }
                break;
            case TRAJECTORY_3:
                if (!drive.isBusy()){
                    drive.followTrajectoryAsync(traj3_2);
                    currentState = State.TRAJECTORY_3_2;
                }
                break;
            case TRAJECTORY_3_2:
                break;
            case TRAJECTORY_4:
                if (System.currentTimeMillis() > waitTime) {
                    traj4 = drive.trajectoryBuilder(pipePose)
                            .lineToLinearHeading(new Pose2d(45, -11, Math.toRadians(0)))
                            .addDisplacementMarker(() -> {
                                rightSlide.setTargetPosition(200);
                                leftSlide.setTargetPosition(200);
                                leftArmServo.setPosition(0);
                                rightArmServo.setPosition(1);
                                clawServo.setPosition(0.24);
                            })
                            .build();
                    currentState = State.TRAJECTORY_5;
                    drive.followTrajectoryAsync(traj4);
                }
                break;
            case TRAJECTORY_5:
                if(!drive.isBusy()) {
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToConstantHeading(new Vector2d(50, -15))
                            .addDisplacementMarker(() -> {
                                clawServo.setPosition(0.38);
                                isConeHoming = true;
                            })
                            .build();
                    currentState = State.HOMING;
                    drive.followTrajectory(traj5);
                }
                break;
            case TO_POLL:
                if(!drive.isBusy()){
                    toPoll = drive.trajectorySequenceBuilder(toPollCenter.end())
                            .lineToLinearHeading(pipePose)
                            .addDisplacementMarker(() -> {
                                clawServo.setPosition(0.38);
                                coneCount++;
                            })
                            .build();
                    currentState = State.POLL_TO_CENTER;
                    drive.followTrajectorySequenceAsync(toPoll);
            }
                break;

            case TO_CONE:
                if (!drive.isBusy()) {
                    clawServo.setPosition(0.38);
                    telemetry.addData("toConeCenter", toConeCenter);
                    toCone = drive.trajectorySequenceBuilder(toConeCenter.end())
                            .lineToLinearHeading(conePose)
                            .addDisplacementMarker(() -> {
                                waitTime = System.currentTimeMillis() + 500;
                            })
                            .build();
                    currentState = State.CONE_TO_CENTER;
                    drive.followTrajectorySequenceAsync(toCone);
                }
                break;
            case CONE_TO_CENTER:
                if (!drive.isBusy()) {
                    clawServo.setPosition(0.24);
                    if (waitTime < System.currentTimeMillis()) {
                        toPollCenter = drive.trajectorySequenceBuilder(conePose)
                                .addTemporalMarker(0.4, () -> {
                                    rightSlide.setTargetPosition(750);
                                    leftSlide.setTargetPosition(750);
                                })
                                .addDisplacementMarker(() -> {
                                    leftArmServo.setPosition(.72);
                                    rightArmServo.setPosition(.3);
                                })
                                .lineToConstantHeading(new Vector2d(36, -12))
                                .turn(Math.toRadians(-45), 5, 5)
                                .build();
                        currentState = State.TO_POLL;
                        drive.followTrajectorySequenceAsync(toPollCenter);
                    }
                }
                break;
            case POLL_TO_CENTER:
                if (!drive.isBusy()) {
                    clawServo.setPosition(0.24);
                    if (waitTime < System.currentTimeMillis()) {
                        toPollCenter = drive.trajectorySequenceBuilder(pipePose)
                                .addTemporalMarker(0.4, () -> {
                                    rightSlide.setTargetPosition(750);
                                    leftSlide.setTargetPosition(750);
                                })
                                .addDisplacementMarker(() -> {
                                    leftArmServo.setPosition(.72);
                                    rightArmServo.setPosition(.3);
                                })
                                .lineToConstantHeading(new Vector2d(36, -12))
                                .turn(Math.toRadians(45), 5, 5)
                                .build();
                        currentState = State.TO_POLL;
                        drive.followTrajectorySequenceAsync(toPollCenter);
                    }
                }
                break;


            case PARK:
                if (!drive.isBusy()) {
                    telemetry.addData("color", colorDetectString);
                    telemetry.update();
                    park2 = drive.trajectorySequenceBuilder(centerTraj.end())
                            .lineToConstantHeading(new Vector2d(36, -37))
                            .build();

                    park3 = drive.trajectorySequenceBuilder(centerTraj.end())
                            .lineToConstantHeading(new Vector2d(60, -37))
                            .build();
                    if (colorDetected == "M") {
                        currentState = State.IDLE;
                        break;
                    } else if (colorDetected == "G") {
                        drive.followTrajectorySequenceAsync(park2);
                        currentState = State.IDLE;
                        break;
                    } else if (colorDetected == "A") {
                        drive.followTrajectorySequenceAsync(park3);
                        currentState = State.IDLE;
                        break;
                    }
                }
                break;
            case IDLE:
                if (!drive.isBusy()){
                    stop();
                }
                // Do nothing in IDLE
                // currentState does not change once in IDLE
                // This concludes the autonomous program
                break;
        }

        drive.update();

        if(isHoming) {
            homePipe(); // Calls method - Locate and position to pipe
        }

        if(isConeHoming){
            homeCone();
        }

        telemetry.update();
    }

    class pipeDetect extends OpenCvPipeline{
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat thresh = new Mat();
        Mat dilate = new Mat();
        Mat hierarchy = new Mat();

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Scalar lowHSV = new Scalar(15,70,70);
            Scalar highHSV = new Scalar(28,255,255);

            Core.inRange(HSV, lowHSV, highHSV, thresh);

            Rect leftRect = new Rect(leftTarget, 140, 1, 79);
            Rect rightRect = new Rect(rightTarget, 140, 1, 79);


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
                lBounding = boundRect[maxValIdx].x;

                rBounding = boundRect[maxValIdx].x + boundRect[maxValIdx].width;

                Imgproc.rectangle(outPut, boundRect[maxValIdx], new Scalar(150, 80, 100), 3);
            }

            Imgproc.rectangle(outPut, leftRect, new Scalar(150, 80, 100), 2);
            Imgproc.rectangle(outPut, rightRect, new Scalar(150, 80, 100), 2);

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

    class coneDetect extends OpenCvPipeline {

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
            Rect leftRect = new Rect(leftTargetCone, 140, 1, 79);
            Rect rightRect = new Rect(rightTargetCone, 140, 1, 79);
            Rect leftBound = new Rect(leftLowBoundCone, 140, 1, 79);
            Rect rightBound = new Rect(rightHighBoundCone, 140, 1, 79);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSV = new Scalar(172,80,80);
            Scalar highHSV = new Scalar(184,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)

            Core.inRange(HSV, lowHSV, highHSV, thresh);


            Imgproc.morphologyEx(thresh, dilate, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(3, 3)));

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
                conexBounding = boundRect[maxValIdx].x;

                coneyBounding = boundRect[maxValIdx].x + boundRect[maxValIdx].width;

                Imgproc.rectangle(outPut, boundRect[maxValIdx], new Scalar(150, 80, 100), 3);
            }


            //copies thresh Mat to outPut and draws rectangles regions on the image for user to see

            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor1, 2);
            Imgproc.rectangle(outPut, leftBound, rectColor1, 2);
            Imgproc.rectangle(outPut, rightBound, rectColor1, 2);


            //Returns to display outPut Mat
            return outPut;
        }
    }
}