package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "MainAutonomous", group = "Auto")
public class MainAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);

        telemetry.addData("状態", "準備OK。ロボットを動かさず待ってください");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        ElapsedTime total = new ElapsedTime();
        drive.driveTo(30,0,0);
        drive.spinTo(270);
        drive.driveTo(30,0,0);


        double used = total.seconds();

        drive.stop();

        while (opModeIsActive()) {
            telemetry.addData("所要時間 (s)", "%.2f", used);
            telemetry.addData("センサー向き (deg)", "%.1f  (0付近なら元に戻っている)", drive.getHeading());
            telemetry.addData("センサーX (in)", "%.2f", drive.getX());
            telemetry.addData("センサーY (in)", "%.2f", drive.getY());
            telemetry.update();
            sleep(20);
        }
    }
}