package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MainAutonomous", group = "Auto")
public class MainAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);

        telemetry.addData("状態", "準備OK。ロボットを動かさず待ってください");
        telemetry.update();

        waitForStart();

        boolean ok = drive.driveTo(40, 0, 0, 0.5, 2.0);

        while (opModeIsActive()) {
            double x = drive.getX();
            double y = drive.getY();
            double h = drive.getHeading();
            telemetry.addData("結果", ok ? "OK" : "TIMEOUT");
            telemetry.addData("センサーX (in)", "%.2f  (%.1f cm)", x, x * 2.54);
            telemetry.addData("センサーY (in)", "%.2f  (%.1f cm)", y, y * 2.54);
            telemetry.addData("センサー向き (deg)", "%.1f", h);
            telemetry.update();
        }
    }
}