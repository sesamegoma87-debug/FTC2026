package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Main Autonomous")
public class MainAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);

        telemetry.addData("Status", "Ready. Press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ============================================================
        // 最初は1つずつテストする。使わない行は // でコメントアウトしておく。
        // ============================================================

        // テスト1: 直進(24インチ進むか、定規で実測する → TICKS_PER_INCHの調整)
        drive.driveStraight(35.4, 0.5);

        drive.turnToHeading(-180);
        telemetry.addData("Status", "Auto Complete");
        telemetry.update();
    }
}