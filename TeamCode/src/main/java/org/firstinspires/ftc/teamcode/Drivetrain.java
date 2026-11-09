package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * メカナム4輪 + goBILDA Pinpoint の走行クラス。
 * 座標系: X=前, Y=左, heading=反時計回りが+(原点 = このクラスを作った地点・向き)
 *
 *  driveTo(x, y, heading)  : 絶対座標へ移動(台形プロファイル + FF + PD)
 *  driveBy(前, 左, 旋回)    : 今の位置からの相対移動
 *  driveStraight(inch, p)  : 今の向きのまま直進
 *  turnToHeading(deg)      : その場で旋回(PID)
 */
public class Drivetrain {

    private final DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final GoBildaPinpointDriver odo;
    private final VoltageSensor voltageSensor;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    private double poseX, poseY, poseHeadingDeg;   // updatePose() のキャッシュ(インチ/度)

    // ================= Pinpoint 設定(実測値) =================
    static final double POD_X_OFFSET_MM = 94.0;     // Xポッド(左側): 回転中心から左が+ (mm)
    static final double POD_Y_OFFSET_MM = 158.5;    // Yポッド(右側): 回転中心から前が+ (mm)
    static final boolean X_POD_REVERSED = false;    // 前に押してXが減るなら true
    static final boolean Y_POD_REVERSED = true;     // 左に押してYが減るなら true

    // ================= フィードフォワード(FeedforwardTunerで測定) =================
    // パワー = KS * 向き + KV * 速度
    static double KV_F = 0.0120, KS_F = 0.069;        // 前後  (KV: power / (in/s))
    static double KV_L = 0.0166, KS_L = 0.111;        // 横
    static double KV_W = 0.0021, KS_W = 0.105;        // 旋回  (KV: power / (deg/s))
    static double NOMINAL_VOLTAGE = 12.88;            // 測定時のバッテリー電圧

    // ================= プロファイル =================
    static double VEL_FRACTION = 0.45;              // 最高速 = このパワー割合で出る速度
    static double ACCEL_TIME = 0.90;                // 最高速まで加速する秒数(大きいほどゆるやか)
    static double TURN_ACCEL_TIME = 0.70;

    // ================= フィードバック(プロファイルとの誤差を補正) =================
    static double POS_KP = 0.045;                   // power / in
    static double POS_KD = 0.012;                   // power / (in/s)   速度誤差
    static double HDG_KP = 1.0 / 60.0;              // power / deg
    static double HDG_KD = 0.002;                   // power / (deg/s)
    static double VEL_FILTER = 0.3;                 // 速度ローパス

    // ================= 終端処理 =================
    static double BREAKAWAY_POWER = 0.08;           // 止まっているときの最低パワー
    static double STUCK_SPEED_IN = 1.0;             // これ未満なら「止まっている」
    static double STUCK_RATE_DEG = 5.0;
    static double SETTLE_SPEED_IN = 1.5;            // 終了判定の速度条件
    static double SETTLE_RATE_DEG = 5.0;
    static double SETTLE_TIME = 0.15;
    static double DEFAULT_POS_TOL_IN = 1.0;
    static double DEFAULT_HEAD_TOL_DEG = 2.0;

    // ================= driveStraight(従来どおり) =================
    static final double ACCEL_DISTANCE_IN = 13.4;
    static final double DECEL_DISTANCE_IN = 13.4;
    static final double MIN_SPEED_FACTOR = 0.2;
    static final double HEADING_CORRECTION_GAIN = 0.02;
    static final double HEADING_INTEGRAL_GAIN = 0.002;
    static final double HEADING_INTEGRAL_MAX = 15;
    static final double MAX_CORRECTION = 0.3;
    static final double CORRECTION_POWER_RATIO = 0.5;
    static final double HEADING_DEADBAND_DEG = 0.5;

    // ================= turnToHeading(従来どおり) =================
    static final double TURN_KP = 1.0 / 40.0;
    static final double TURN_KI = 0.0005;
    static final double TURN_KD = 0.01;
    static final double TURN_INTEGRAL_MAX = 5;
    static final double TURN_MIN_POWER = 0.25;
    static final double TURN_TOLERANCE_DEG = 2.0;
    static final double TURN_SETTLE_RATE = 5.0;
    static final double TURN_SETTLE_TIME = 0.15;
    static final double TURN_MAX_ACCEL = 0.10;
    static final double RATE_FILTER = 0.3;

    // ================= 安全装置 =================
    static final double DRIVE_TIMEOUT_SEC = 10;
    static final double TURN_TIMEOUT_SEC = 5;

    public Drivetrain(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(19.61, DistanceUnit.MM);     // 手押し3回の平均で補正
        odo.setEncoderDirections(
                X_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Y_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();        // ロボットを静止させた状態で
        opMode.sleep(300);           // リセット完了待ち
        updatePose();
    }

    // =====================================================================
    //  driveTo : 台形プロファイル + フィードフォワード + PD
    // =====================================================================
    public boolean driveTo(double targetX, double targetY, double targetHeadingDeg) {
        return driveTo(targetX, targetY, targetHeadingDeg, DEFAULT_POS_TOL_IN, DEFAULT_HEAD_TOL_DEG);
    }

    /** @return true = 到達して静止 / false = タイムアウトまたは停止 */
    public boolean driveTo(double targetX, double targetY, double targetHeadingDeg,
                           double posTolIn, double headTolDeg) {
        final double voltScale = clamp(NOMINAL_VOLTAGE / Math.max(getVoltage(), 1.0), 0.8, 1.4);

        updatePose();
        final double startX = poseX, startY = poseY, startH = poseHeadingDeg;
        final double D = Math.hypot(targetX - startX, targetY - startY);
        final double ux = D > 1e-6 ? (targetX - startX) / D : 0;     // 進行方向(フィールド座標)
        final double uy = D > 1e-6 ? (targetY - startY) / D : 0;
        final double dH = shortestAngleDifference(startH, targetHeadingDeg);   // 目標 − 開始
        final double absDH = Math.abs(dH);
        final double hSign = Math.signum(dH);

        // 進行方向(開始姿勢でのロボット基準)から速度・加速度の上限を決める。
        double vMax = 1, aMax = 1;
        if (D > 1e-6) {
            double h0 = Math.toRadians(startH);
            double uF = ux * Math.cos(h0) + uy * Math.sin(h0);
            double uL = -ux * Math.sin(h0) + uy * Math.cos(h0);
            vMax = VEL_FRACTION / (Math.abs(uF) * KV_F + Math.abs(uL) * KV_L);
            aMax = vMax / ACCEL_TIME;
        }
        final double wMax = VEL_FRACTION / KV_W;
        final double wAcc = wMax / TURN_ACCEL_TIME;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime dtTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        double lastX = startX, lastY = startY, lastH = startH;
        double vx = 0, vy = 0, w = 0;     // フィールド座標の速度 / 角速度(反時計回り+)

        while (opMode.opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SEC) {
            double dt = Math.max(dtTimer.seconds(), 1e-3);
            dtTimer.reset();
            double t = timer.seconds();

            updatePose();
            double h = Math.toRadians(poseHeadingDeg);
            double c = Math.cos(h), s = Math.sin(h);

            // 実測速度(差分 + ローパス)
            vx = (1 - VEL_FILTER) * vx + VEL_FILTER * (poseX - lastX) / dt;
            vy = (1 - VEL_FILTER) * vy + VEL_FILTER * (poseY - lastY) / dt;
            w  = (1 - VEL_FILTER) * w  + VEL_FILTER * shortestAngleDifference(lastH, poseHeadingDeg) / dt;
            lastX = poseX; lastY = poseY; lastH = poseHeadingDeg;

            // 参照 {位置, 速度, フェーズ}
            double[] pr = trapezoid(D, vMax, aMax, t);
            double[] hr = trapezoid(absDH, wMax, wAcc, t);
            double refX = startX + ux * pr[0], refY = startY + uy * pr[0];
            double refVx = ux * pr[1], refVy = uy * pr[1];
            double refH = startH + hSign * hr[0];
            double refW = hSign * hr[1];

            // 減速中はKS(摩擦分の押し)を切る。切らないと止まる直前まで押してしまい行き過ぎる
            double ksP = pr[2] < 0 ? 0 : 1;
            double ksW = hr[2] < 0 ? 0 : 1;

            // 終了判定(最終目標に対する誤差 + 静止)
            double dist = Math.hypot(targetX - poseX, targetY - poseY);
            double headErr = shortestAngleDifference(poseHeadingDeg, targetHeadingDeg);
            boolean posOk = dist < posTolIn;
            boolean headOk = Math.abs(headErr) < headTolDeg;
            double speed = Math.hypot(vx, vy);
            boolean slow = speed < SETTLE_SPEED_IN && Math.abs(w) < SETTLE_RATE_DEG;
            if (!(posOk && headOk && slow)) {
                settleTimer.reset();
            } else if (settleTimer.seconds() > SETTLE_TIME) {
                stop();
                return true;
            }

            // --- 並進: フィードフォワード(ロボット基準で軸別) ---
            double refVf = refVx * c + refVy * s;
            double refVl = -refVx * s + refVy * c;
            double ffF = 0, ffL = 0;
            if (pr[1] > 1e-6) {
                ffF = ksP * KS_F * (refVf / pr[1]) + KV_F * refVf;
                ffL = ksP * KS_L * (refVl / pr[1]) + KV_L * refVl;
            }
            // --- 並進: フィードバック(フィールド座標で誤差を出してロボット基準へ) ---
            double fbX = POS_KP * (refX - poseX) + POS_KD * (refVx - vx);
            double fbY = POS_KP * (refY - poseY) + POS_KD * (refVy - vy);
            double f = ffF * voltScale + (fbX * c + fbY * s);
            double l = ffL * voltScale + (-fbX * s + fbY * c);

            // --- 旋回 ---
            double ffW = hr[1] > 1e-6 ? hSign * KS_W * ksW + KV_W * refW : 0;
            double tn = ffW * voltScale
                    + HDG_KP * shortestAngleDifference(poseHeadingDeg, refH)
                    + HDG_KD * (refW - w);

            // プロファイル完了後は、許容内の成分を0にする(最低パワーで押し出さない)
            boolean profileDone = pr[0] >= D - 1e-6 && hr[0] >= absDH - 1e-6;
            if (profileDone) {
                if (posOk) { f = 0; l = 0; }
                if (headOk) tn = 0;
            }

            double fl = f - l - tn, fr = f + l + tn, bl = f + l - tn, br = f - l + tn;
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));
            double scale = 1.0;
            if (max > 1.0) {
                scale = 1.0 / max;
            } else if (profileDone && speed < STUCK_SPEED_IN && Math.abs(w) < STUCK_RATE_DEG
                    && max > 1e-6 && max < BREAKAWAY_POWER) {
                scale = BREAKAWAY_POWER / max;      // 止まっているときだけ摩擦突破
            }
            setWheelPowers(fl * scale, fr * scale, bl * scale, br * scale);

            // 調整用テレメトリ
            double progress = (poseX - startX) * ux + (poseY - startY) * uy;
            double cross = -(poseX - startX) * uy + (poseY - startY) * ux;
            telemetry.addData("--- driveTo ---", "t=%.2f s  volt=x%.2f", t, voltScale);
            telemetry.addData("Ref v / Actual v (in/s)", "%.1f / %.1f", pr[1], speed);
            telemetry.addData("Along err (ref-actual, in)", "%.2f", pr[0] - progress);
            telemetry.addData("Cross err (in)", "%.2f", cross);
            telemetry.addData("Ref w / Actual w (deg/s)", "%.1f / %.1f", refW, w);
            telemetry.addData("Dist / HeadErr", "%.2f in / %.2f deg", dist, headErr);
            telemetry.addData("Power boost (1.00=なし)", "%.2f", scale);
            telemetry.update();
        }

        stop();
        telemetry.addData("driveTo", "TIMEOUT");
        telemetry.update();
        return false;
    }

    /** 今の位置・向きからの相対移動(前, 左, 反時計回りの旋回)。誤差は累積する。 */
    public boolean driveBy(double forwardIn, double leftIn, double turnDeg) {
        updatePose();
        double h = Math.toRadians(poseHeadingDeg);
        double tx = poseX + forwardIn * Math.cos(h) - leftIn * Math.sin(h);
        double ty = poseY + forwardIn * Math.sin(h) + leftIn * Math.cos(h);
        return driveTo(tx, ty, poseHeadingDeg + turnDeg);
    }

    /**
     * 台形(距離が短いときは三角)プロファイル。
     * 戻り値 {位置, 速度, フェーズ}  フェーズ: +1=加速, 0=巡航/完了, -1=減速
     */
    private static double[] trapezoid(double D, double vMax, double aMax, double t) {
        if (D < 1e-9 || t <= 0) return new double[]{0, 0, 0};
        double tAcc = vMax / aMax;
        double dAcc = 0.5 * aMax * tAcc * tAcc;
        double vPeak = vMax, tCruise;
        if (2 * dAcc > D) {                       // 最高速に届かない → 三角
            vPeak = Math.sqrt(D * aMax);
            tAcc = vPeak / aMax;
            dAcc = D / 2;
            tCruise = 0;
        } else {
            tCruise = (D - 2 * dAcc) / vMax;
        }
        double total = 2 * tAcc + tCruise;
        if (t < tAcc) return new double[]{0.5 * aMax * t * t, aMax * t, 1};
        if (t < tAcc + tCruise) return new double[]{dAcc + vPeak * (t - tAcc), vPeak, 0};
        if (t < total) {
            double td = total - t;
            return new double[]{D - 0.5 * aMax * td * td, aMax * td, -1};
        }
        return new double[]{D, 0, 0};
    }

    // =====================================================================
    //  driveStraight : 台形制御 + Pinpoint補正(従来どおり)
    // =====================================================================
    public void driveStraight(double inches, double maxPower) {
        updatePose();
        double startX = poseX, startY = poseY, startHeading = poseHeadingDeg;
        double ux = Math.cos(Math.toRadians(startHeading));
        double uy = Math.sin(Math.toRadians(startHeading));
        double targetDist = Math.abs(inches);
        double sign = Math.signum(inches);
        double headingIntegral = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime dtTimer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SEC) {
            double dt = dtTimer.seconds();
            dtTimer.reset();

            updatePose();
            double progress = ((poseX - startX) * ux + (poseY - startY) * uy) * sign;
            double remaining = targetDist - progress;
            if (remaining <= 0) break;
            double traveled = Math.max(0, progress);

            double speedFactor;
            if (traveled < ACCEL_DISTANCE_IN) speedFactor = traveled / ACCEL_DISTANCE_IN;
            else if (remaining < DECEL_DISTANCE_IN) speedFactor = remaining / DECEL_DISTANCE_IN;
            else speedFactor = 1.0;
            speedFactor = Math.max(MIN_SPEED_FACTOR, Math.min(1.0, speedFactor));
            double power = sign * maxPower * speedFactor;

            double headingError = shortestAngleDifference(startHeading, poseHeadingDeg); // 現在 − 開始
            double correction = 0;
            if (Math.abs(headingError) >= HEADING_DEADBAND_DEG) {
                headingIntegral = clamp(headingIntegral + headingError * dt,
                        -HEADING_INTEGRAL_MAX, HEADING_INTEGRAL_MAX);
                double limit = Math.min(MAX_CORRECTION, Math.abs(power) * CORRECTION_POWER_RATIO);
                double raw = headingError * HEADING_CORRECTION_GAIN
                        + headingIntegral * HEADING_INTEGRAL_GAIN;
                correction = clamp(raw, -limit, limit);
            } else {
                headingIntegral = 0;
            }

            setDrivePower(power + correction, power - correction);

            telemetry.addData("--- Drive Straight ---", "");
            telemetry.addData("Target / Progress (in)", "%.1f / %.1f", targetDist, progress);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Heading Error (deg)", "%.2f", headingError);
            telemetry.update();
        }
        stop();
    }

    // =====================================================================
    //  turnToHeading : PID + 摩擦突破 + 加速度制限(従来どおり)
    // =====================================================================
    public void turnToHeading(double targetAngle) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        ElapsedTime dtTimer = new ElapsedTime();
        double integral = 0, lastPower = 0, rate = 0;

        updatePose();
        double lastHeading = poseHeadingDeg;

        while (opMode.opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SEC) {
            double dt = dtTimer.seconds();
            dtTimer.reset();

            updatePose();
            double currentHeading = poseHeadingDeg;
            double error = shortestAngleDifference(currentHeading, targetAngle);

            double rawRate = shortestAngleDifference(lastHeading, currentHeading) / Math.max(dt, 1e-3);
            rate = (1 - RATE_FILTER) * rate + RATE_FILTER * rawRate;
            lastHeading = currentHeading;

            boolean settled = Math.abs(error) < TURN_TOLERANCE_DEG && Math.abs(rate) < TURN_SETTLE_RATE;
            if (!settled) settleTimer.reset();
            if (settleTimer.seconds() > TURN_SETTLE_TIME) break;

            if (Math.abs(error) >= TURN_TOLERANCE_DEG) {
                integral = clamp(integral + error * dt, -TURN_INTEGRAL_MAX, TURN_INTEGRAL_MAX);
            } else {
                integral = 0;
            }

            double targetPower = error * TURN_KP + integral * TURN_KI - rate * TURN_KD;
            if (Math.abs(error) > TURN_TOLERANCE_DEG && Math.abs(error) < 5.0) {
                double dir = (targetPower != 0) ? Math.signum(targetPower) : Math.signum(error);
                targetPower = dir * Math.max(Math.abs(targetPower), TURN_MIN_POWER);
            }
            targetPower = clamp(targetPower, -1.0, 1.0);
            targetPower = clamp(targetPower, lastPower - TURN_MAX_ACCEL, lastPower + TURN_MAX_ACCEL);

            setDrivePower(-targetPower, targetPower);
            lastPower = targetPower;

            telemetry.addData("--- Turn To Heading ---", "");
            telemetry.addData("Target / Current", "%.1f / %.2f", targetAngle, currentHeading);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Rate (deg/s)", "%.1f", rate);
            telemetry.addData("Power", "%.2f", targetPower);
            telemetry.update();
        }

        stop();
        opMode.sleep(300);
        telemetry.addData("Turn Complete", "Final Heading: %.2f", getHeading());
        telemetry.update();
    }

    // =====================================================================
    //  公開ヘルパー(チューナーや相対移動で使う)
    // =====================================================================
    public void updatePose() {
        odo.update();
        Pose2D p = odo.getPosition();
        poseX = p.getX(DistanceUnit.INCH);
        poseY = p.getY(DistanceUnit.INCH);
        poseHeadingDeg = p.getHeading(AngleUnit.DEGREES);
    }
    public double getX() { updatePose(); return poseX; }
    public double getY() { updatePose(); return poseY; }
    public double getHeading() { updatePose(); return poseHeadingDeg; }
    public double getVoltage() { return voltageSensor.getVoltage(); }

    /** ロボット基準のパワー指定。f=前+, l=左+, t=反時計回り+ */
    public void setRobotPower(double f, double l, double t) {
        double fl = f - l - t, fr = f + l + t, bl = f + l - t, br = f - l + t;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        setWheelPowers(fl / max, fr / max, bl / max, br / max);
    }
    public void stop() { setWheelPowers(0, 0, 0, 0); }

    /** to - from を -180〜180度で返す */
    static double shortestAngleDifference(double fromAngle, double toAngle) {
        double diff = Math.toRadians(toAngle - fromAngle);
        return Math.toDegrees(Math.atan2(Math.sin(diff), Math.cos(diff)));
    }

    // ---- 内部 ----
    private void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(clamp(fl, -1, 1));
        frontRightMotor.setPower(clamp(fr, -1, 1));
        backLeftMotor.setPower(clamp(bl, -1, 1));
        backRightMotor.setPower(clamp(br, -1, 1));
    }
    private void setDrivePower(double left, double right) {
        setWheelPowers(left, right, left, right);
    }
    private static double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
}