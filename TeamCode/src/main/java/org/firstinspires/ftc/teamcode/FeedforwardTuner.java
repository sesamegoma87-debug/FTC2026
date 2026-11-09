package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tune: Feedforward", group = "Tune")
public class FeedforwardTuner extends LinearOpMode {
    static final double RUN_SEC = 0.9;        // 1回の走行時間
    static final double WINDOW_START = 0.5;   // 加速が終わってから測る

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, this, telemetry);
        String result = "A=前後 / B=横 / X=旋回 を押すと測定開始(前後に約90cm空ける)";
        waitForStart();

        while (opModeIsActive()) {
            int axis = gamepad1.a ? 0 : gamepad1.b ? 1 : gamepad1.x ? 2 : -1;
            if (axis >= 0) {
                result = measure(drive, axis);
                while (opModeIsActive() && (gamepad1.a || gamepad1.b || gamepad1.x)) idle();
            }
            telemetry.addLine(result);
            telemetry.update();
        }
    }

    private String measure(Drivetrain drive, int axis) {
        double[] powers = (axis == 2) ? new double[]{0.20, 0.30, 0.40}
                : new double[]{0.30, 0.45, 0.60};
        String unit = (axis == 2) ? "deg/s" : "in/s";
        String name = axis == 0 ? "前後(KV_F, KS_F)" : axis == 1 ? "横(KV_L, KS_L)" : "旋回(KV_W, KS_W)";
        double volt = drive.getVoltage();

        double[] v = new double[powers.length];
        StringBuilder pts = new StringBuilder();
        for (int i = 0; i < powers.length; i++) {
            v[i] = sample(drive, axis, powers[i]);
            pts.append(String.format("  power %.2f -> %.1f %s\n", powers[i], v[i], unit));
        }

        // 最小二乗: power = KS + KV * v
        double n = powers.length, sv = 0, sp = 0;
        for (int i = 0; i < n; i++) { sv += v[i]; sp += powers[i]; }
        double mv = sv / n, mp = sp / n, num = 0, den = 0;
        for (int i = 0; i < n; i++) { num += (v[i] - mv) * (powers[i] - mp); den += (v[i] - mv) * (v[i] - mv); }
        if (den < 1e-9) return "測定失敗: ほとんど動いていません(配線・向き・最低パワーを確認)";
        double kv = num / den, ks = mp - kv * mv;

        String warn = v[0] < 1.0 ? "\n※最小パワーでほぼ動いていません。KSが過大に出ます" : "";
        return String.format("【%s】\n%s  KV = %.4f\n  KS = %.3f\n  最高速(推定) = %.1f %s\n  電圧 = %.2f V (NOMINAL_VOLTAGEに設定)%s",
                name, pts, kv, ks, (1.0 - ks) / kv, unit, volt, warn);
    }

    /** +power と -power で走って、平均の速さ(in/s または deg/s)を返す */
    private double sample(Drivetrain drive, int axis, double power) {
        double sum = 0;
        for (int dir = 1; dir >= -1 && opModeIsActive(); dir -= 2) {
            double p = power * dir;
            drive.setRobotPower(axis == 0 ? p : 0, axis == 1 ? p : 0, axis == 2 ? p : 0);

            ElapsedTime t = new ElapsedTime();
            double x0 = 0, y0 = 0, h0 = 0, t0 = 0;
            boolean started = false;
            while (opModeIsActive() && t.seconds() < RUN_SEC) {
                double x = drive.getX(), y = drive.getY(), h = drive.getHeading();  // 毎ループ更新
                if (!started && t.seconds() >= WINDOW_START) {
                    x0 = x; y0 = y; h0 = h; t0 = t.seconds(); started = true;
                }
            }
            double x1 = drive.getX(), y1 = drive.getY(), h1 = drive.getHeading();
            double dt = t.seconds() - t0;
            drive.stop();

            double dist = (axis == 2) ? Math.abs(Drivetrain.shortestAngleDifference(h0, h1))
                    : Math.hypot(x1 - x0, y1 - y0);
            sum += dist / Math.max(dt, 1e-3);
            sleep(400);
        }
        return sum / 2.0;
    }
}