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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

@Autonomous(name = "Auto Program 1 + 5 Epic Auto", group = "Main")

public class AutoProgram5Cones extends OpMode {

    //region Variable Declaration
    SampleMecanumDrive drive;

    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");
    int red;

    TrajectorySequence firstToJunc;
    TrajectorySequence firstToCone;
    Trajectory startConeHoming;


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
    double waitTime2;
    double currentTime;

    double parkTimer;
    boolean parkSetup = false;

    WebcamName webcamNameFront;

    int cameraMonitorViewId;

    String colorDetectString = "";
    String colorDetected = "";

    double lowestY = 150;
    Pose2d posEst;

    int coneCount = 0;

    double lBoundingPole;
    double rBoundingPole;

    double lBoundingCone;
    double rBoundingCone;

    int leftTargetPole = 275;
    int rightTargetPole = 395;

    int leftTargetCone = 180;
    int rightTargetCone = 450;

    double baseSpeedVert = 0.01;
    double baseSpeedHorz = 0.01;
    double motorVertical;
    double motorHorizontal;
    double multiplier = 0.25;
    int hAOffset = 150;
    int vAOffset = 100;
    int tOffset = 15;

    boolean isHoming = false;
    boolean isConeHoming = false;
    boolean homed = false;
    boolean coneGrabbed = false;


    enum State {
        FIRST_TO_JUNC,
        FIRST_TO_CONE,
        START_CONE_HOME,
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

    //endregion


    public void homeCone() {
        telemetry.update();
        double coneWidth = rBoundingCone - lBoundingCone;
        double targetWidth = rightTargetCone - leftTargetCone;

        //find center of cones and target
        double coneCenter = lBoundingCone + (coneWidth / 2);
        double targetCenter = leftTargetCone + (targetWidth / 2);

        //get center distance
        double cOffset = targetCenter - coneCenter;

        double wOffset = targetWidth - coneWidth;

            if (red >= 200 ){
                clawServo.setPosition(0.24);
                isConeHoming = false;
                drive.setDrivePower(new Pose2d(0, 0, 0));
                drive.update();
                conePose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(0));

                waitTime = System.currentTimeMillis() + 200;
                waitTime2 = waitTime + 300;

                currentState = State.CONE_TO_CENTER;
            }

            else if (isConeHoming) {

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

                if(Math.abs(wOffset) > vAOffset) {
                    motorVertical = multiplier + baseSpeedVert;
                }
                else if(Math.abs(wOffset) > tOffset) {
                    motorVertical = multiplier * (wOffset / vAOffset) + baseSpeedVert;
                }
                else{
                    motorVertical = 0;
                }

                if(Math.abs(cOffset) > hAOffset) {
                    motorHorizontal = multiplier + baseSpeedHorz;
                }
                else if(Math.abs(cOffset) > tOffset) {
                    motorHorizontal = multiplier * (cOffset / hAOffset) + baseSpeedHorz;
                }
                else{
                    motorHorizontal = 0;
                }
                drive.setDrivePower(new Pose2d(motorVertical, motorHorizontal, 0));
            }



        telemetry.addData("cOff", cOffset);
        telemetry.addData("wOff", wOffset);
        telemetry.addData("motorHorz", motorHorizontal);
        telemetry.addData("motorVerti", motorVertical);
        telemetry.addData("baseHorz", baseSpeedHorz);
        telemetry.addData("baseVert", baseSpeedVert);

            telemetry.update();
    }
    public void homeJunc() {

        //find width of target and poles
        double poleWidth = rBoundingPole - lBoundingPole;
        double targetWidth = rightTargetPole - leftTargetPole;

        //find center of poles and target
        double poleCenter = lBoundingPole + (poleWidth / 2);
        double targetCenter = leftTargetPole + (targetWidth / 2);

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


        if(Math.abs(wOffset) > vAOffset) {
            motorVertical = multiplier + baseSpeedVert;
        }
        else if(Math.abs(wOffset) > tOffset) {
            motorVertical = multiplier * (wOffset / vAOffset) + baseSpeedVert;
        }
        else{
            motorVertical = 0;
        }

        if(Math.abs(cOffset) > hAOffset) {
            motorHorizontal = multiplier + baseSpeedHorz;
        }
        else if(Math.abs(cOffset) > tOffset) {
            motorHorizontal = multiplier * (cOffset / hAOffset) + baseSpeedHorz;
        }
        else{
            motorHorizontal = 0;
        }

        if (Math.abs(wOffset) <= tOffset && Math.abs(cOffset) <= tOffset){
            isHoming = false;

            drive.setDrivePower(new Pose2d(0, 0, 0));
            drive.update();
            pipePose = drive.getPoseEstimate();


            clawServo.setPosition(0.38);
            homed = true;
            currentTime = System.currentTimeMillis();
            waitTime = System.currentTimeMillis() + 500.0;
            currentState = State.FIRST_TO_CONE;
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
    }


    // based on time and power which determines turn direction and turn speed
    @Override
    public void init() {

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");

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
                webcam.setPipeline(new colorDetect());
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
                webcamfront.setPipeline(new coneDetect());
                webcamfront.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Starts streaming camera


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(36, -63.96875, Math.toRadians(270.0)));

        firstToJunc = drive.trajectorySequenceBuilder(new Pose2d(36, -63.96875, Math.toRadians(270.0)))
                .addDisplacementMarker(() -> {
                    colorDetected = colorDetectString;
                    telemetry.addData("color", colorDetectString);
                    telemetry.update();
                })
                .addDisplacementMarker(() -> {
                    webcam.setPipeline(new pipeDetect());
                    rightSlide.setTargetPosition(750);
                    leftSlide.setTargetPosition(750);
                })
                .setVelConstraint(drive.getVelocityConstraint(1000, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToSplineHeading(new Pose2d(35.90, -9.93, Math.toRadians(300.00)))
                .addDisplacementMarker(() -> isHoming = true)
                .build();


        currentState = State.FIRST_TO_CONE;

        drive.followTrajectorySequenceAsync(firstToJunc);

            telemetry.addData("Status", "Initialized");
            telemetry.update();
    }

    public void loop(){

        red = colorSensor.red();

        if(!parkSetup){
            parkTimer = System.currentTimeMillis() + 27000;
            parkSetup = true;
        }
        else if(currentState != State.IDLE && currentState != State.PARK && parkTimer < System.currentTimeMillis()){

                toPollCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> {
                            clawServo.setPosition(0.24);
                            leftArmServo.setPosition(0);
                            rightArmServo.setPosition(1);
                        })
                        .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                        .addTemporalMarker(0.4, () -> {
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);
                            clawServo.setPosition(0.38);
                        })
                        .build();
                currentState = State.PARK;
                drive.followTrajectorySequenceAsync(toPollCenter);
        }


        label:
        switch (currentState) {
            case FIRST_TO_CONE:
                if (System.currentTimeMillis() > waitTime) {
                    firstToCone = drive.trajectorySequenceBuilder(pipePose)
                            .lineToSplineHeading(new Pose2d(44.64, -12.29, Math.toRadians(0.00)))
                            .addDisplacementMarker(() -> {
                                rightSlide.setTargetPosition(185);
                                leftSlide.setTargetPosition(185);
                                leftArmServo.setPosition(0);
                                rightArmServo.setPosition(1);
                                clawServo.setPosition(0.24);
                            })
                            .build();
                    currentState = State.START_CONE_HOME;
                    drive.followTrajectorySequenceAsync(firstToCone);
                }
                break;
            case START_CONE_HOME:
                if(!drive.isBusy()) {
                    startConeHoming = drive.trajectoryBuilder(firstToCone.end())
                            .addDisplacementMarker(() -> {
                                clawServo.setPosition(0.38);
                                isConeHoming = true;
                            })
                            .build();
                    currentState = State.HOMING;
                    drive.followTrajectory(startConeHoming);
                }
                break;
            case TO_POLL:
                if(!drive.isBusy() && red < 200){
                    toCone = drive.trajectorySequenceBuilder(toPollCenter.end())
                            .addDisplacementMarker(() -> {
                                clawServo.setPosition(0.38);
                                leftArmServo.setPosition(0);
                                rightArmServo.setPosition(1);
                                rightSlide.setTargetPosition(185 - (37 * coneCount));
                                leftSlide.setTargetPosition(185 - (37 * coneCount));
                            })
                            .lineToLinearHeading(conePose)
                            .addDisplacementMarker(() -> waitTime = System.currentTimeMillis() + 2500)
                            .build();
                    currentState = State.CONE_TO_CENTER;
                    drive.followTrajectorySequenceAsync(toCone);
                }

                else if(!drive.isBusy()){
                    toPoll = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addDisplacementMarker( () -> coneCount++)
                            .turn(Math.toRadians(-45), 5, 5)
                            .lineToLinearHeading(pipePose)
                            .addDisplacementMarker(() -> waitTime = System.currentTimeMillis() + 2000)
                            .build();
                    currentState = State.POLL_TO_CENTER;
                    drive.followTrajectorySequenceAsync(toPoll);
            }
                break;

            case TO_CONE:
                if (!drive.isBusy()) {
                    clawServo.setPosition(0.38);
                    telemetry.addData("toConeCenter", toConeCenter);
                    toCone = drive.trajectorySequenceBuilder(toPollCenter.end())
                            .lineToLinearHeading(conePose)
                            .addDisplacementMarker(() -> waitTime = System.currentTimeMillis() + 2500)
                            .build();
                    currentState = State.CONE_TO_CENTER;
                    drive.followTrajectorySequenceAsync(toCone);
                }
                break;

            case CONE_TO_CENTER:
                if (!drive.isBusy()) {
                    if (red < 200 && !coneGrabbed)
                    drive.setDrivePower(new Pose2d(0.2, 0, 0));
                    else {
                        coneGrabbed = true;
                        drive.setDrivePower(new Pose2d(0, 0, 0));
                        clawServo.setPosition(0.24);
                        drive.setPoseEstimate(posEst);


                        if (waitTime < System.currentTimeMillis()) {
                            coneGrabbed = false;
                            rightSlide.setTargetPosition(800);
                            leftSlide.setTargetPosition(800);
                            toPollCenter = drive.trajectorySequenceBuilder(conePose)
                                    .lineToConstantHeading(new Vector2d(36, -12), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .addDisplacementMarker(() -> {
                                        leftArmServo.setPosition(.72);
                                        rightArmServo.setPosition(.3);
                                    })
                                    .build();
                            currentState = State.TO_POLL;
                            drive.followTrajectorySequenceAsync(toPollCenter);
                        }
                    }
                }
                break;

            case POLL_TO_CENTER:
                if (!drive.isBusy()) {
                    clawServo.setPosition(0.38);
                    if (waitTime < System.currentTimeMillis()) {
                        toPollCenter = drive.trajectorySequenceBuilder(pipePose)
                                .addDisplacementMarker(() -> {
                                    clawServo.setPosition(0.24);
                                    leftArmServo.setPosition(0);
                                    rightArmServo.setPosition(1);
                                })
                                .lineToConstantHeading(new Vector2d(36, -12))
                                .addTemporalMarker(0.4, () -> {
                                    rightSlide.setTargetPosition(185 - (37 * coneCount));
                                    leftSlide.setTargetPosition(185 - (37 * coneCount));
                                })
                                .turn(Math.toRadians(45), 5, 5)
                                .build();
                        if(coneCount < 2) {
                            currentState = State.TO_CONE;
                        }
                        else{
                            currentState = State.PARK;
                        }
                        drive.followTrajectorySequenceAsync(toPollCenter);
                    }
                }
                break;


            case PARK:
                if (!drive.isBusy()) {
                    telemetry.addData("color", colorDetectString);
                    telemetry.update();
                    rightSlide.setTargetPosition(0);
                    leftSlide.setTargetPosition(0);
                    clawServo.setPosition(0.38);
                    park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(12, -12))
                            .build();

                    park3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(60, -12))
                            .build();
                    switch (colorDetected) {
                        case "M":
                            drive.followTrajectorySequenceAsync(park2);
                            currentState = State.IDLE;
                            break label;
                        case "G":
                            currentState = State.IDLE;
                            break label;
                        case "A":
                            drive.followTrajectorySequenceAsync(park3);
                            currentState = State.IDLE;
                            break label;
                    }
                }
                break;
            case IDLE:
                if (!drive.isBusy()){
                    stop();
                }
                break;
        }
        if(!isHoming) {
            drive.update();
        }

        if(isHoming) {
            homeJunc(); // Calls method - Locate and position to pipe
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

            Rect leftRect = new Rect(leftTargetPole, 140, 1, 79);
            Rect rightRect = new Rect(rightTargetPole, 140, 1, 79);


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
                lBoundingPole = boundRect[maxValIdx].x;

                rBoundingPole = boundRect[maxValIdx].x + boundRect[maxValIdx].width;

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
            Scalar lowHSVM = new Scalar(152,40,40);
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

        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat thresh = new Mat();
        Mat dilate = new Mat();
        Mat hierarchy = new Mat();

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Scalar lowHSV = new Scalar(162,40,40);
            Scalar highHSV = new Scalar(190,255,255);

            Core.inRange(HSV, lowHSV, highHSV, thresh);

            Rect leftRect = new Rect(leftTargetCone, 140, 1, 79);
            Rect rightRect = new Rect(rightTargetCone, 140, 1, 79);


            Imgproc.morphologyEx(thresh, dilate, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(2, 2)));

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
                lBoundingCone = boundRect[maxValIdx].x;

                rBoundingCone = boundRect[maxValIdx].x + boundRect[maxValIdx].width;

                Imgproc.rectangle(outPut, boundRect[maxValIdx], new Scalar(150, 80, 100), 3);
            }

            Imgproc.rectangle(outPut, leftRect, new Scalar(150, 80, 100), 2);
            Imgproc.rectangle(outPut, rightRect, new Scalar(150, 80, 100), 2);

            return outPut;
        }
    }
}