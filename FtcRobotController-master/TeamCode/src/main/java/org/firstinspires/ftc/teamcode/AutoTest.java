package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.MORPH_DILATE;
import static org.opencv.imgproc.Imgproc.MORPH_ERODE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name="Auto Test", group="Concept")
public class AutoTest extends OpMode {
    OpenCvWebcam webcam = null;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontRight;
    DcMotorEx frontLeft;

    double xBounding = 0;
    double yBounding = 0;

    double motorSpeed = 1000;

    double leftavgfin;
    double rightavgfin;
    double centeravgfin;

    boolean isHoming = true;

    double direction;
    double magnitude;
    double fRight;
    double bRight;
    double bLeft;
    double fLeft;
    double turn;

    @Override
    public void init(){
        //region Hardware Map
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //endregion

        //gets webcam name and Id from configfile and initializes OpenCvWebcam object "webcam"
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //sets webcam Computer Vision Pipeline to examplePipeline
        webcam.setPipeline(new examplePipeline());

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
    }

    @Override
    public void loop(){

//        motorSpeed = (Math.abs(xBounding - 270) + Math.abs(yBounding - 380)) * 6 + 100;
        motorSpeed = 500;

        if (xBounding > 250 && xBounding < 270 && yBounding > 380 && yBounding < 400){
            isHoming = false;
            magnitude = 0;
            direction = 0;
            turn = 0;

        }
        if (isHoming) {
            if (xBounding > 250 && yBounding < 400) {
                    direction = -1.57;
                    magnitude = 1;
                    turn = 0;

                }
            else if (xBounding > 270 && yBounding > 380) {
                    turn = .2;
                }
            else if (xBounding < 270 && yBounding < 380) {
                    turn = -.2;
                }
            else if (xBounding < 250 && yBounding > 400) {
                    direction = -1.57;
                    magnitude = -1;
                    turn = 0;
                }
        }

        fRight = (motorSpeed * (-Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
        bLeft = (motorSpeed * (Math.sin(direction - 1.0 / 4.0 * Math.PI) * magnitude + turn));
        bRight = (motorSpeed * (-Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));
        fLeft = (motorSpeed * (Math.sin(direction + 1.0 / 4.0 * Math.PI) * magnitude + turn));

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


        telemetry.update();
    }

    class examplePipeline extends OpenCvPipeline {

        //Initializing Variables
        Mat HSV = new Mat();
        Mat outPut = new Mat();

        Scalar rectColor1 = new Scalar(50, 255, 255);

        //Loops and processes every frame and returns desired changed
        public Mat processFrame(Mat input) {

            //changes Mat input from RGB to HSV and saves to Mat HSV
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            //Creates New rectangle objects for 3 regions, Left, Right, and Center
            Rect leftRect = new Rect(270, 140, 1, 79);
            Rect rightRect = new Rect(380, 140, 1, 79);
            Rect leftBound = new Rect(250, 140, 1, 79);
            Rect rightBound = new Rect(400, 140, 1, 79);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSV = new Scalar(16,50,50);
            Scalar highHSV = new Scalar(24,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)
            Mat thresh = new Mat();
            Core.inRange(HSV, lowHSV, highHSV, thresh);

            Mat dilate = new Mat();
            Imgproc.morphologyEx(thresh, dilate, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_RECT, new Size(7, 7)));

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
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
                Imgproc.rectangle(outPut, boundRect[maxValIdx], new Scalar(300, 80, 100), 3);
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
}