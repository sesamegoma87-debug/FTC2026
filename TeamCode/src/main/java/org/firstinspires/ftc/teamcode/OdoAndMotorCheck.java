package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** 切り分け用。A=前進 / Y=左ストレーフ / B=その場CCW / 十字キー=単輪。無入力ならモーター0。 */
@TeleOp(name = "CHECK_OdoAndMotors", group = "Test")
public class OdoAndMotorCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor fr = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor bl = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor br = hardwareMap.dcMotor.get("backRightMotor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }
        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(Drivetrain.POD_X_OFFSET_MM, Drivetrain.POD_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(Drivetrain.ENCODER_TICKS_PER_MM, DistanceUnit.MM);
        odo.setEncoderDirections(
                Drivetrain.X_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Drivetrain.Y_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        sleep(1000);                       // ★ 300ms → 1000ms（IMUキャリブ完了待ち）
        odo.update();

        telemetry.addLine("START後: 何も押さず手で機体を動かして x/y/heading を確認");
        telemetry.update();
        waitForStart();

        ElapsedTime dtTimer = new ElapsedTime();
        double lastH = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double minDt = 1.0, maxDt = 0.0;

        while (opModeIsActive()) {
            double dt = Math.max(dtTimer.seconds(), 1e-4);
            dtTimer.reset();
            minDt = Math.min(minDt, dt);
            maxDt = Math.max(maxDt, dt);

            odo.update();
            Pose2D p = odo.getPosition();
            double x = p.getX(DistanceUnit.INCH);
            double y = p.getY(DistanceUnit.INCH);
            double h = p.getHeading(AngleUnit.DEGREES);
            double hRate = (h - lastH) / dt;   // 静止時のノイズ量を見る用(±180跨ぎは無視)
            lastH = h;

            final double P = 0.30;
            double pFl = 0, pFr = 0, pBl = 0, pBr = 0;
            String mode = "OFF: 手で押して odo を見る";
            if (gamepad1.a) {
                pFl = P; pFr = P; pBl = P; pBr = P; mode = "FORWARD 0.30";
            } else if (gamepad1.y) {
                pFl = -P; pFr = P; pBl = P; pBr = -P; mode = "STRAFE LEFT 0.30";
            } else if (gamepad1.b) {
                pFl = -P; pFr = P; pBl = -P; pBr = P; mode = "SPIN CCW 0.30";
            } else if (gamepad1.dpad_up)    { pFl = P; mode = "FL only +0.30"; }
            else if (gamepad1.dpad_right)   { pFr = P; mode = "FR only +0.30"; }
            else if (gamepad1.dpad_left)    { pBl = P; mode = "BL only +0.30"; }
            else if (gamepad1.dpad_down)    { pBr = P; mode = "BR only +0.30"; }

            fl.setPower(pFl); fr.setPower(pFr); bl.setPower(pBl); br.setPower(pBr);

            telemetry.addData("mode", mode);
            telemetry.addData("x / y (in)", "%.2f / %.2f", x, y);
            telemetry.addData("heading (deg)", "%.2f", h);
            telemetry.addData("heading rate (deg/s)", "%.1f  静止で±5以上ならノイズ", hRate);
            telemetry.addData("dt min / max (ms)", "%.1f / %.1f", minDt * 1000, maxDt * 1000);
            telemetry.addData("battery (V)", "%.2f", vs.getVoltage());
            telemetry.addData("power fl/fr/bl/br", "%.2f %.2f %.2f %.2f", pFl, pFr, pBl, pBr);
            telemetry.update();
        }
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }
}