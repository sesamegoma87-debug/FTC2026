package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tune: Pose Check", group = "Tune")
public class PoseCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);
        telemetry.addLine("Init完了。STARTで開始");
        telemetry.update();
        waitForStart();

        double x0 = 0, y0 = 0, lastH = 0, turned = 0;   // 基準点と累積回転角
        lastH = drive.getHeading();

        while (opModeIsActive()) {
            double x = drive.getX(), y = drive.getY(), h = drive.getHeading();
            turned += Drivetrain.shortestAngleDifference(lastH, h);   // 360°を超えても数えられる
            lastH = h;

            if (gamepad1.a) {                // A: 基準をリセット(ここから測り直し)
                x0 = x; y0 = y; turned = 0;
            }

            // B長押し: モーターでその場旋回(反時計回り・低速)
            if (gamepad1.b) drive.setRobotPower(0, 0, 0.3);
            else drive.stop();

            telemetry.addLine("A=基準リセット / B長押し=その場で反時計回りに旋回");
            telemetry.addData("X (前+) in", "%.2f", x);
            telemetry.addData("Y (左+) in", "%.2f", y);
            telemetry.addData("Heading deg", "%.1f", h);
            telemetry.addData("--- 基準からの変化 ---", "");
            telemetry.addData("累積回転 deg", "%.0f  (%.1f 周)", turned, turned / 360.0);
            telemetry.addData("ΔX / ΔY in", "%.2f / %.2f", x - x0, y - y0);
            telemetry.addData("ずれ in", "%.2f", Math.hypot(x - x0, y - y0));
            telemetry.update();
        }
        drive.stop();
    }
}