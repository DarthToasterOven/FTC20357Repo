package org.firstinspires.ftc.teamcode.Toros.Autonomous;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous(name = "Autonomous Right")
public class AutoV2 extends LinearOpMode {

    public class Claw {
        private Servo claw;

        public Claw (HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class closeClaw implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                claw.setPosition(1); //What our close position is
                return false;
            }
        }
        public class openClaw implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                claw.setPosition(0); //What our open position is
                return false;
            }
        }
        public Action ClawOpen(){
            return new openClaw();
        }
        public Action ClawClose(){
            return new closeClaw();
        }
    }


    public class Arm {

        private DcMotorEx joint1, joint2, Slide;

        public Arm(HardwareMap hardwareMap) {
            joint1 = hardwareMap.get(DcMotorEx.class, "joint1");
            joint2 = hardwareMap.get(DcMotorEx.class, "joint2");
            Slide = hardwareMap.get(DcMotorEx.class, "extend");
        }

        public class PIDFj1 {
            private PIDController controller;

            public double p1 = -0.04, i1 = 0.001, d1 = 0.0005;

            private double f1 = 0.185;

            private final double ticks_in_degrees = 1440 / 180;

            public double calc(int target1) {
                controller = new PIDController(p1, i1, d1);
                controller.setPID(p1, i1, d1);
                int armPos = joint1.getCurrentPosition();
                double pid = controller.calculate(armPos, target1);
                double ff = Math.cos(Math.toRadians(target1 / ticks_in_degrees)) * f1;

                double power = pid + ff;
                return power;
            }
        }

            public class PIDFj2 {
                private PIDController controller;

                public double p1 = -0.04, i1 = 0.001, d1 = 0.0005;

                private double f1 = 0.185;

                private final double ticks_in_degrees = 1440 / 180;

                public double calc(int target1) {
                    controller = new PIDController(p1, i1, d1);
                    controller.setPID(p1, i1, d1);
                    int armPos = joint1.getCurrentPosition();
                    double pid = controller.calculate(armPos, target1);
                    double ff = Math.cos(Math.toRadians(target1 / ticks_in_degrees)) * f1;

                    double power = pid + ff;
                    return power;
                }

            }

            public class joint1Up implements Action {
                private boolean initialized = false;
                PIDFj1 p = new PIDFj1();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        p.calc(500);//Our wanted position of the first arm
                        initialized = true;
                    }

                    double pos = joint1.getCurrentPosition();
                    packet.put("Arm pos", pos);
                    if (pos < 490) {
                        return true;
                    } else {
                        return false;
                    }
                }

            }

            public class joint1Down implements Action {
                private boolean initialized = false;
                PIDFj1 p = new PIDFj1();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        p.calc(250);//Our wanted position of the first arm
                        initialized = true;
                    }

                    double pos = joint1.getCurrentPosition();
                    packet.put("Arm pos", pos);
                    if (pos > 250) {
                        return true;
                    } else {
                        return false;
                    }
                }
            }

            public class joint2Up implements Action {
                private boolean initialized = false;
                PIDFj2 p = new PIDFj2();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        p.calc(500);//Our wanted position of the first arm
                        initialized = true;
                    }

                    double pos = joint2.getCurrentPosition();
                    packet.put("Arm pos", pos);
                    if (pos < 490) {
                        return true;
                    } else {
                        return false;
                    }
                }
            }

            public class joint2Down implements Action {
                private boolean initialized = false;
                PIDFj2 p = new PIDFj2();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        p.calc(250);//Our wanted position of the first arm
                        initialized = true;
                    }

                    double pos = joint2.getCurrentPosition();
                    packet.put("Arm pos", pos);
                    if (pos > 250) {
                        return true;
                    } else {
                        return false;
                    }
                }
            }

            public class SlideUp implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        Slide.setPower(0.8);
                        initialized = true;
                    }
                    double pos = Slide.getCurrentPosition();
                    packet.put("Slide", pos);
                    if (pos < 300) { //Or whatever position
                        return true;
                    } else {
                        Slide.setPower(0);
                        return false;
                    }
                }
            }

            public class SlideDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        Slide.setPower(-0.8);
                        initialized = true;
                    }
                    double pos = Slide.getCurrentPosition();
                    packet.put("Slide", pos);
                    if (pos > 100) { //Or whatever position
                        return true;
                    } else {
                        Slide.setPower(0);
                        return false;
                    }
                }
            }

            public Action joint1up() {
                return new joint1Up();
            }

            public Action joint2up() {
                return new joint2Up();
            }

            public Action joint1down() {
                return new joint1Down();
            }

            public Action joint2down() {
                return new joint2Down();
            }

            public Action slideup() {
                return new SlideUp();
            }

            public Action slidedown() {
                return new SlideDown();
            }
        }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(24,-61,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder sample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8,-33))
                .waitSeconds(1);
        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(-40)
                .setTangent(Math.toRadians(350))
                .splineToLinearHeading(new Pose2d(48,-10,Math.toRadians(270)),Math.toRadians(270))
                .strafeTo(new Vector2d(48,-52))
                .strafeTo(new Vector2d(48,-15))
                .strafeTo(new Vector2d(58,-15))
                .strafeTo(new Vector2d(58,-52))
                .strafeTo(new Vector2d(58,-15))
                .strafeTo(new Vector2d(64,-15))
                .strafeTo(new Vector2d(64,-52));
        Action trjEnd = traj.endTrajectory().fresh()
                .build();
//        TrajectoryActionBuilder firstTraj = drive.actionBuilder(drive.pose)

        waitForStart();

        if(isStopRequested()) return;

       Action action1 = sample.build();
        Action action2 = traj.build();
//Parallel Actions?
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(claw.ClawClose())
                        .waitSeconds(5)
                        .stopAndAdd(claw.ClawOpen())
                        .waitSeconds(3)
                        .stopAndAdd(arm.joint1up())
                        .build()
        );
    }
}
