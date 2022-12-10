package oldprogram;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class RoadRunnerTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(41, -63.96875, Math.toRadians(270.0)));

        waitForStart();

        if (isStopRequested()) return;

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

        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

        sleep(2000);
    }
}