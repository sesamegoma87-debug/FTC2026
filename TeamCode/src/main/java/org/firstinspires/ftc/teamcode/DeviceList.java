package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp(name = "Tune: Device List", group = "Tune")
public class DeviceList extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            for (HardwareDevice d : hardwareMap.getAll(HardwareDevice.class)) {
                for (String n : hardwareMap.getNamesOf(d)) {
                    telemetry.addData("[" + n + "]", d.getDeviceName());
                }
            }
            telemetry.update();
        }
    }
}