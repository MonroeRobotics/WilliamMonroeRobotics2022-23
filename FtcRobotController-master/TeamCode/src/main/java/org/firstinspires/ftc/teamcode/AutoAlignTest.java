package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name="Auto Align Test")
public class AutoAlignTest extends OpMode {
    OpenCvWebcam webcam;
    int cameraMonitorViewId;

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


    SampleMecanumDrive drive;

    public void init(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //sets webcam Computer Vision Pipeline to examplePipeline
                webcam.setPipeline(new AutoAlignTest.pipeDetect());
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Init Done");

    }

    public void loop(){
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
}
