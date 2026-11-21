package org.firstinspires.ftc.teamcode;

// SDK 10.1 以降。それ以前は自前でコピーした GoBildaPinpointDriver を import
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 第一弾: odo の X/Y/heading から、フィールド上の固定点を常に狙う。
 * スタート地点 = (0,0)、スタート時のロボット向き = 0、タレットも 0 基準。
 * 第2弾で getTargetFieldAngle() をカメラのトラッキング結果に差し替える。
 *
 * 改良点(ぎこちなさ対策):
 *  - KS: 静止摩擦を越える最小パワーを足す(目標に近づくにつれ滑らかに 0 へ)
 *  - D項にローパスフィルタ(エンコーダ tick の離散ノイズ対策)
 *  - I項を実際に効く大きさにし、符号反転でリセット(ワインドアップ防止)
 *  - テレメトリ更新を間引き、制御ループを軽く保つ
 */
@TeleOp(name = "TurretOdoTracker", group = "Test")
public class TurretOdoTracker extends LinearOpMode {

    // ===== フィールド座標 [mm]（スタート地点基準）=====
    static final double TARGET_X_MM = 1200.0;
    static final double TARGET_Y_MM = 0.0;

    // ===== オドメトリポッドの取り付けオフセット [mm]（回転中心から）=====
    static final double POD_X_OFFSET_MM = 0.0;
    static final double POD_Y_OFFSET_MM = 0.0;

    // ===== タレット =====
    // モーター出力軸の tick/rev × 外部減速比（実測推奨: 手で1周させて値を読む）
    static final double TURRET_TICKS_PER_REV = 537.7 * 3.0;
    static final int    TURRET_DIRECTION     = +1;          // 逆に回るなら -1

    // 可動範囲 [rad]（スタート=0基準、ケーブル巻き込み防止）
    static final double TURRET_MIN_RAD = Math.toRadians(-90.0);
    static final double TURRET_MAX_RAD = Math.toRadians(+90.0);

    // ===== PID =====
    static final double KP = 1.2;
    static final double KI = 0.5;
    static final double KD = 0.05;
    static final double I_CLAMP = 0.1;      // 積分の最大寄与 = KI × I_CLAMP = 0.05
    static final double MAX_POWER = 0.8;

    // ===== 静止摩擦補償・フィルタ =====
    static final double KS = 0.08;                          // 動き出す最小パワー（要実測）
    static final double FADE_RAD = Math.toRadians(4.0);     // KS を 0 へ戻していく幅
    static final double D_FILTER = 0.8;                     // 0〜1、大きいほど滑らか
    static final double DEADBAND_RAD = Math.toRadians(0.7);

    // テレメトリ更新間隔 [s]
    static final double TELEMETRY_INTERVAL_S = 0.1;

    private DcMotorEx turret;
    private GoBildaPinpointDriver odo;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double dFiltered = 0.0;
    private final ElapsedTime dtTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret_motor"); // Configure Robot の名前と一致させる
        odo    = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // スタート向き = 0
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // エンコーダは読むだけ
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        // ---- odo 設定 ----
        odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addLine("IMU キャリブレーション中... ロボットを動かさないで");
        telemetry.update();

        odo.resetPosAndIMU();   // ここが (0,0,0)。約1秒かかる
        sleep(1500);
        odo.update();

        telemetry.addLine("Ready. ロボットをスタート位置・スタート向きに置いて START");
        telemetry.addData("odo status", odo.getDeviceStatus());
        telemetry.update();

        waitForStart();
        dtTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            odo.update();

            double x       = odo.getPosX(DistanceUnit.MM);
            double y       = odo.getPosY(DistanceUnit.MM);
            double heading = odo.getHeading(AngleUnit.RADIANS);

            // 1) ターゲットのフィールド絶対方位
            double targetFieldAngle = getTargetFieldAngle(x, y);

            // 2) ロボット座標系での目標タレット角
            double desiredTurret = wrap(targetFieldAngle - heading);

            // 3) 可動範囲チェック（超えたら端で止める）
            boolean reachable = desiredTurret >= TURRET_MIN_RAD && desiredTurret <= TURRET_MAX_RAD;
            double setpoint   = clamp(desiredTurret, TURRET_MIN_RAD, TURRET_MAX_RAD);

            // 4) 現在のタレット角（スタート = 0）
            double turretAngle = TURRET_DIRECTION
                    * turret.getCurrentPosition() / TURRET_TICKS_PER_REV * 2.0 * Math.PI;

            // 5) PID + 静止摩擦補償
            double dt = Math.max(dtTimer.seconds(), 1e-3);
            dtTimer.reset();

            double error  = wrap(setpoint - turretAngle);
            double absErr = Math.abs(error);
            double power  = 0.0;

            if (absErr < DEADBAND_RAD) {
                integral = 0.0;
                dFiltered = 0.0;
            } else {
                // 誤差の符号が反転したら積分をリセット（オーバーシュート後の引きずり防止）
                if (error * lastError < 0) integral = 0.0;
                integral = clamp(integral + error * dt, -I_CLAMP, I_CLAMP);

                double rawD = (error - lastError) / dt;
                dFiltered = D_FILTER * dFiltered + (1.0 - D_FILTER) * rawD;

                // デッドバンドの縁で 0、FADE_RAD 以上で KS 全開
                double ks = KS * clamp((absErr - DEADBAND_RAD) / FADE_RAD, 0.0, 1.0);

                power = KP * error + KI * integral + KD * dFiltered + Math.signum(error) * ks;
                power = clamp(power, -MAX_POWER, MAX_POWER);
            }
            lastError = error;

            // 端で外向きに回さない（ケーブル保護）
            if ((turretAngle >= TURRET_MAX_RAD && power > 0)
                    || (turretAngle <= TURRET_MIN_RAD && power < 0)) {
                power = 0.0;
                integral = 0.0;
            }

            turret.setPower(TURRET_DIRECTION * power);

            // ---- テレメトリ（間引き）----
            if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_S) {
                telemetryTimer.reset();
                telemetry.addData("odo x/y [mm]", "%.0f / %.0f", x, y);
                telemetry.addData("heading [deg]", "%.1f", Math.toDegrees(heading));
                telemetry.addData("target field [deg]", "%.1f", Math.toDegrees(targetFieldAngle));
                telemetry.addData("desired turret [deg]", "%.1f", Math.toDegrees(desiredTurret));
                telemetry.addData("turret [deg]", "%.1f", Math.toDegrees(turretAngle));
                telemetry.addData("error [deg]", "%.2f", Math.toDegrees(error));
                telemetry.addData("power", "%.2f", power);
                telemetry.addData("reachable", reachable ? "OK" : "OUT OF RANGE");
                telemetry.addData("odo status", odo.getDeviceStatus());
                telemetry.update();
            }
        }

        turret.setPower(0);
    }

    /** スタート原点から見たターゲットの絶対方位 [rad]。第2弾はここをカメラに置換。 */
    private double getTargetFieldAngle(double x, double y) {
        return Math.atan2(TARGET_Y_MM - y, TARGET_X_MM - x);
    }

    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}