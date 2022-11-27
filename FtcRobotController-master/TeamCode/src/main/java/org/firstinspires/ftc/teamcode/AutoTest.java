package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

@Autonomous(name="Auto Test", group="Concept")
public class AutoTest extends OpMode {
    OpenCvWebcam webcam = null;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontRight;
    DcMotorEx frontLeft;

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
        if (centeravgfin < 240){
            motorSpeed = (-3)*centeravgfin + 1000;
            direction = -1.57;
            magnitude = 1;
            if(leftavgfin > 0){
                turn = -.2;
            }
            if(rightavgfin > 0){
                turn = .2;
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
                turn = .2;
            }
            else if(leftavgfin > 240){
                motorSpeed = 500;
                direction = 0;
                magnitude = 0;
                turn = -.2;
            }
            else{
                magnitude = 0;
                direction = 0;
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
            Scalar lowHSV = new Scalar(15,50,50);
            Scalar highHSV = new Scalar(24,255,255);

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