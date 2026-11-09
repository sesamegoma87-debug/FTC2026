package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Tune: Move Test", group = "Tune")
public class MoveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);
        waitForStart();
        if (isStopRequested()) return;

        // 調整中は1つだけ有効にして繰り返す
        test(drive, "前進40",  40, 0, 0);
        test(drive, "戻る",     0, 0, 0);
        // test(drive, "横24",    0, 24, 0);
        // test(drive, "戻る",     0, 0, 0);
        // test(drive, "旋回90",   0, 0, 90);
        // test(drive, "戻る",     0, 0, 0);
        // test(drive, "斜め+旋回", 24, 24, 90);
        // test(drive, "戻る",     0, 0, 0);

        while (opModeIsActive()) idle();   // ログを読めるように待機
    }

    private void test(Drivetrain d, String name, double x, double y, double h) {
        boolean ok = d.driveTo(x, y, h);
        sleep(700);   // 完全に止まってから最終誤差を読む
        telemetry.log().add("%s: %s 誤差 x=%.2f y=%.2f h=%.1f", name, ok ? "OK" : "TIMEOUT",
                x - d.getX(), y - d.getY(), Drivetrain.shortestAngleDifference(d.getHeading(), h));
    }
}