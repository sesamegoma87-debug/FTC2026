package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BIOBUZZ_HiveTeleOp", group = "TeleOp")
public class HiveTeleOp extends OpMode {

    // ===== 🔫 タレット定数 =====
    static final double TURRET_BASE_TICKS_PER_REV = 537.7 * 3.0; // 基準Tick * ギア比
    static final int    TURRET_DIRECTION          = +1;          // 逆に回るなら -1

    // 回転角のスケール補正 (走行中に十字キーの左・右で 0.005 ずつ微調整可能)
    private double turretScale = 0.960;

    // ★★★ 可動範囲: 右270° (-270°) 〜 左90° (+90°) ★★★
    // (この機体では + が左回転、- が右回転のため、これで「左に90°、右に270°」になります)
    static final double TURRET_MIN_RAD = Math.toRadians(-270.0); // 右に270°まで
    static final double TURRET_MAX_RAD = Math.toRadians(90.0);   // 左に90°まで
    // 境界(-270° / +90°)でのチャタリング防止用の遊び角度 (4°)
    static final double WRAP_HYSTERESIS_RAD = Math.toRadians(4.0);

    // ===== 🎛️ PID & 加速度設定（高レスポンス＆行き過ぎ防止） =====
    static final double KP = 1.15;             // 比例ゲイン
    static final double KI = 0.35;             // 積分ゲイン
    static final double KD = 0.085;            // 行き過ぎ(オーバーシュート)を止めるブレーキゲイン
    static final double I_ZONE_RAD = Math.toRadians(8.0); // 8°以内の時だけ積分を有効化(旋回中の溜まり防止)
    static final double I_CLAMP = 0.12;
    static final double MAX_POWER = 0.75;      // 最大パワー
    static final double ACCEL_SLEW_RATE = 3.0; // 加速時のスルーレート(減速時は即ブレーキ)
    static final double KS = 0.07;             // 静止摩擦補償
    static final double FADE_RAD = Math.toRadians(4.0);
    static final double D_FILTER = 0.70;
    static final double DEADBAND_RAD = Math.toRadians(0.5);

    // ===== 🛞 オドメトリポッド定数 ("odo" を使用) =====
    static final String ODO_NAME = "odo";

    static final double ODO_WHEEL_DIAMETER_MM = 48.0;   // ポッドの車輪径 [mm]
    static final double ODO_TICKS_PER_REV     = 8192.0; // ポッド1回転のTick数
    static final double ODO_MM_PER_TICK       = Math.PI * ODO_WHEEL_DIAMETER_MM / ODO_TICKS_PER_REV;
    static final double ODO_SIGN              = +1.0;   // 前進で減る場合は -1.0

    // Hive(0, 0)の近く(30cm以内)では初期方位を維持
    static final double MIN_TRACKING_DIST_MM = 300.0;

    // ===== 🎮 走行スティックの向き補正 =====
    static final double DRIVE_FORWARD_SIGN = +1.0;
    static final double DRIVE_STRAFE_SIGN  = +1.0;
    static final double DRIVE_TURN_SIGN    = +1.0;

    // ===== ハードウェア =====
    private DcMotor fl, bl, fr, br;
    private DcMotorEx turret;
    private DcMotorEx odoPod;
    private boolean hasOdo = false;
    private CRServo sqL, sqR;
    private Shooter shooter;
    private IMU imu;

    // ===== ステート変数 =====
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double hiveX  = 0.0;
    private double hiveY  = 0.0;
    private int lastOdoTicks = 0;
    private double lastHeading = 0.0;

    private double initBaseRad = 0.0;     // INIT時の基準絶対方位 [rad]
    private double targetAbsRad = 0.0;    // フィールド上の絶対目標方位 [rad]
    private double currentSetpoint = 0.0; // 前回のタレット目標角 [rad]

    private boolean firing = false;
    private double shooterPower = 0.8;
    private boolean prevRb, prevU, prevD, prevL, prevR, prevYaw;

    // PID・ソフトスタート用メモリ
    private double integral = 0.0;
    private double lastError = 0.0;
    private double dFiltered = 0.0;
    private double prevTurretPower = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        // --- 走行モーター ---
        fl = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        bl = hardwareMap.get(DcMotor.class, "backLeftMotor");
        fr = hardwareMap.get(DcMotor.class, "frontRightMotor");
        br = hardwareMap.get(DcMotor.class, "backRightMotor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{fl, bl, fr, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }

        // --- タレット ---
        turret = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        // --- オドメトリポッド ("odo") ---
        try {
            odoPod = hardwareMap.get(DcMotorEx.class, ODO_NAME);
            if (odoPod != fl && odoPod != bl && odoPod != fr && odoPod != br) {
                odoPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            lastOdoTicks = odoPod.getCurrentPosition();
            hasOdo = true;
        } catch (Exception e) {
            hasOdo = false;
        }

        // --- Squeeze ---
        sqL = hardwareMap.crservo.get("left_squeeze_motor");
        sqR = hardwareMap.crservo.get("right_squeeze_motor");
        sqL.setDirection(DcMotorSimple.Direction.REVERSE);
        sqL.setPower(0);
        sqR.setPower(0);

        // --- Shooter ---
        shooter = new Shooter(hardwareMap);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // INITを押した場所を Hive の位置 (0, 0) として記憶
        robotX = 0.0;
        robotY = 0.0;
        hiveX  = 0.0;
        hiveY  = 0.0;
        currentSetpoint = 0.0;
    }

    @Override
    public void init_loop() {
        double initHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastHeading = initHeading;
        initBaseRad = initHeading;
        targetAbsRad = initHeading;

        if (hasOdo) {
            lastOdoTicks = odoPod.getCurrentPosition();
        }

        telemetry.addData("状態", "現在地をHive(0, 0)としてロック完了");
        telemetry.addData("タレット可動域", "左90°(+90°) 〜 右270°(-270°)");
        telemetry.addData("オドメトリ (odo)", hasOdo ? "接続OK (Ticks: " + lastOdoTicks + ")" : "未検出");
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        // スティック入力
        double y  = -gamepad1.left_stick_y * DRIVE_FORWARD_SIGN;
        double x  =  gamepad1.left_stick_x * DRIVE_STRAFE_SIGN;
        double rx =  gamepad1.right_stick_x * DRIVE_TURN_SIGN;

        // 1. ヨー角＆Hive位置リセット (Y または Options ボタン)
        boolean yawBtn = gamepad1.options || gamepad1.y;
        if (yawBtn && !prevYaw) {
            imu.resetYaw();
            lastHeading = 0.0;
            robotX = 0.0;
            robotY = 0.0;
            hiveX  = 0.0;
            hiveY  = 0.0;
            initBaseRad = 0.0;
            targetAbsRad = 0.0;
        }
        prevYaw = yawBtn;

        // 十字キー左右でタレットの回転量を 0.005 刻みで微調整可能
        // 行き過ぎるなら「左」、足りないなら「右」
        boolean l = gamepad1.dpad_left, r = gamepad1.dpad_right;
        if (l && !prevL) turretScale = Math.max(0.500, turretScale - 0.005);
        if (r && !prevR) turretScale = Math.min(1.500, turretScale + 0.005);
        prevL = l; prevR = r;

        // 2. 現在のヨー角取得
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double dHeading = wrap(heading - lastHeading);
        lastHeading = heading;
        double yawRateRadPerSec = Math.abs(dHeading / dt);

        // 3. オドメトリ("odo") による自己位置更新
        int currentOdoTicks = 0;
        if (hasOdo) {
            currentOdoTicks = odoPod.getCurrentPosition();
            int deltaTicks = currentOdoTicks - lastOdoTicks;
            lastOdoTicks = currentOdoTicks;

            // 旋回中はポッドの空転による座標ズレを防ぐため移動量に加算しない
            boolean isTurning = (Math.abs(rx) > 0.05) || (yawRateRadPerSec > Math.toRadians(15.0));
            double dForward = isTurning ? 0.0 : (deltaTicks * ODO_MM_PER_TICK * ODO_SIGN);

            double midHeading = heading - dHeading / 2.0;
            robotX += dForward * Math.cos(midHeading);
            robotY += dForward * Math.sin(midHeading);

            double dx = hiveX - robotX;
            double dy = hiveY - robotY;
            double distToHive = Math.hypot(dx, dy);

            if (distToHive > MIN_TRACKING_DIST_MM) {
                targetAbsRad = Math.atan2(dy, dx);
            } else {
                targetAbsRad = initBaseRad;
            }
        }

        // 4. 現在のタレット相対角 (turretScale で回転角の大きさを補正)
        double effectiveTicksPerRev = TURRET_BASE_TICKS_PER_REV * turretScale;
        double turretAngle = TURRET_DIRECTION * turret.getCurrentPosition() / effectiveTicksPerRev * 2.0 * Math.PI;

        // 5. タレットの目標相対角を計算 (右270°[-270°] 〜 左90°[+90°])
        double desiredTurret = targetAbsRad - heading;
        double setpoint = selectTurretSetpoint(desiredTurret, currentSetpoint);
        currentSetpoint = setpoint;

        // 6. エラー計算
        double error = setpoint - turretAngle;

        // 7. PID制御（行き過ぎ防止機能付き）
        double absErr = Math.abs(error);
        double targetPower = 0.0;

        if (absErr < DEADBAND_RAD) {
            integral = 0.0;
            dFiltered = 0.0;
        } else {
            // 目標角の近く(8°以内)の時だけ積分を効かせ、旋回中の積分溜まりによる行き過ぎを防ぐ
            if (error * lastError <= 0 || absErr > I_ZONE_RAD) {
                integral = 0.0;
            } else {
                integral = clamp(integral + error * dt, -I_CLAMP, I_CLAMP);
            }

            double rawD = (error - lastError) / dt;
            dFiltered = D_FILTER * dFiltered + (1.0 - D_FILTER) * rawD;

            double ks = KS * clamp((absErr - DEADBAND_RAD) / FADE_RAD, 0.0, 1.0);
            targetPower = KP * error + KI * integral + KD * dFiltered + Math.signum(error) * ks;
            targetPower = clamp(targetPower, -MAX_POWER, MAX_POWER);
        }
        lastError = error;

        // 加速時のみスルーレート制限をかけ、減速・ブレーキ時は即座に反映してオーバーシュートを防ぐ
        double power;
        boolean isAcceleratingSameDir = (targetPower * prevTurretPower >= 0.0)
                && (Math.abs(targetPower) > Math.abs(prevTurretPower));
        if (isAcceleratingSameDir) {
            double maxStep = ACCEL_SLEW_RATE * dt;
            power = clamp(targetPower, prevTurretPower - maxStep, prevTurretPower + maxStep);
        } else {
            power = targetPower;
        }

        // 可動域端(右270°[-270°] / 左90°[+90°])でのハードリミット保護
        if ((turretAngle >= TURRET_MAX_RAD && power > 0) || (turretAngle <= TURRET_MIN_RAD && power < 0)) {
            power = 0.0;
            integral = 0.0;
        }
        prevTurretPower = power;
        turret.setPower(TURRET_DIRECTION * power);

        // 8. Shooter & Squeeze (RBトグル、十字キー上下で出力調整)
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRb) firing = !firing;
        prevRb = rb;

        boolean u = gamepad1.dpad_up, d = gamepad1.dpad_down;
        if (u && !prevU) shooterPower = Math.min(1.0, shooterPower + 0.05);
        if (d && !prevD) shooterPower = Math.max(0.0, shooterPower - 0.05);
        prevU = u; prevD = d;

        shooter.setPower(firing ? shooterPower : 0.0);
        shooter.update();
        sqL.setPower(firing ? 1.0 : 0.0);
        sqR.setPower(firing ? 1.0 : 0.0);

        // 9. 走行
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= 1.1;
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        fl.setPower((rotY + rotX + rx) / den);
        bl.setPower((rotY - rotX + rx) / den);
        fr.setPower((rotY - rotX - rx) / den);
        br.setPower((rotY + rotX - rx) / den);

        // Telemetry
        telemetry.addData("Pos (X, Y) [mm]", "(%.0f, %.0f) | Odo: %d", robotX, robotY, currentOdoTicks);
        telemetry.addData("Robot Yaw [deg]", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Turret [deg]", "%.1f / Target: %.1f (右-270〜左+90)", Math.toDegrees(turretAngle), Math.toDegrees(setpoint));
        telemetry.addData("Turret Scale (十字左右)", "%.3f (左:行き過ぎ減 / 右:回転増)", turretScale);
        telemetry.addData("Turret Power", "%.2f", power);
        telemetry.addData("Firing", "%s (Power: %.2f)", firing ? "ON" : "OFF", shooterPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        fl.setPower(0); bl.setPower(0); fr.setPower(0); br.setPower(0);
        turret.setPower(0); sqL.setPower(0); sqR.setPower(0); shooter.setPower(0);
    }

    // --- ヘルパーメソッド ---
    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /**
     * 目標角度を [右270°(-270°), 左90°(+90°)] に変換する
     */
    private static double selectTurretSetpoint(double desiredRad, double prevSetpoint) {
        double unwrapped = prevSetpoint + wrap(desiredRad - prevSetpoint);

        // [-270°, +90°] の範囲内ならそのまま使用
        if (unwrapped >= TURRET_MIN_RAD && unwrapped <= TURRET_MAX_RAD) {
            return unwrapped;
        }

        // 端(-270° / +90°)での微小ノイズによるチャタリング防止
        if (unwrapped > TURRET_MAX_RAD && unwrapped <= TURRET_MAX_RAD + WRAP_HYSTERESIS_RAD) {
            return TURRET_MAX_RAD;
        }
        if (unwrapped < TURRET_MIN_RAD && unwrapped >= TURRET_MIN_RAD - WRAP_HYSTERESIS_RAD) {
            return TURRET_MIN_RAD;
        }

        // 端を超えたら反対側(-270° ⇔ +90°)へ折り返す
        double wrapped = unwrapped;
        while (wrapped < TURRET_MIN_RAD) wrapped += 2.0 * Math.PI;
        while (wrapped > TURRET_MAX_RAD) wrapped -= 2.0 * Math.PI;
        return clamp(wrapped, TURRET_MIN_RAD, TURRET_MAX_RAD);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}