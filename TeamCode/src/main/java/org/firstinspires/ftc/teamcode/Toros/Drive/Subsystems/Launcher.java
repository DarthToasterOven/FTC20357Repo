package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private DcMotorEx launch;
    private Gamepad gamepad1;
    private Servo gate;
    boolean toggle = false;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad) {
        gamepad1 = gamepad;
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void runLauncher() {
        if (gamepad1.dpadUpWasPressed()) {
            toggle = !toggle;
        }
        if (gamepad1.right_trigger > 0.1) {
            launch.setPower(-gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.1) {
            launch.setPower(gamepad1.left_trigger/4); // wrong way
        }
        else{
            launch.setPower(0);
        }
        if (toggle) {
            launch.setPower(-1);
        } else {
            launch.setPower(0);
        }
        if (gamepad1.b) {
            launch.setPower(0);
        }



    }
    public double getLauncherSpeed(){
       return launch.getVelocity();
    }
}
