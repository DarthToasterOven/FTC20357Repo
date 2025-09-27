package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Auto2025 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagsys aprilTagsys = new AprilTagsys(hardwareMap, gamepad2);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                aprilTagsys.runAprilTags();


                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.

            }
        }

        // Save more CPU resources when camera is no longer needed.

    }


}




