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
        boolean o1 = drive.spinBy(90);    // 反時計回りに +90°
        boolean o2 = drive.spinBy(-90);   // 元の向きに戻る(-90°)
        boolean o3 = drive.spinBy(180);   // +180°
        boolean o4 = drive.spinBy(-180);  // 元に戻る(-180°)
        double used = total.seconds();

        drive.stop();

        while (opModeIsActive()) {
            telemetry.addData("結果 90 / -90 / 180 / -180", "%s / %s / %s / %s",
                    o1 ? "OK" : "NG", o2 ? "OK" : "NG", o3 ? "OK" : "NG", o4 ? "OK" : "NG");
            telemetry.addData("所要時間 (s)", "%.2f", used);
            telemetry.addData("センサー向き (deg)", "%.1f  (0付近なら元に戻っている)", drive.getHeading());
            telemetry.addData("センサーX (in)", "%.2f", drive.getX());
            telemetry.addData("センサーY (in)", "%.2f", drive.getY());
            telemetry.update();
            sleep(20);
        }
    }
}