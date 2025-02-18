package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@Config
@Autonomous(name = "Left Auto", group = "ITD Auto", preselectTeleOp = "AyMarthaV2")
public class LeftSideTestAuto extends LinearOpMode {
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 1000;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.15;
    final double OuttakeElbowPositionIn = 0.85;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeElbowPositionOutSpecimen = 0.1;
    //final double OuttakeElbowPositionMiddle = 0.55;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private enum IntakeState{
        IN,
        OUT
    }
    private Servo IntakeClaw;

    public static double IntakeSliderPositionIN = 0.73;
    final double IntakeSliderPositionOut = 0.35;
    final double IntakeElbowPositionIn = 0.95;
    final double IntakeElbowPositionOut = 0.0;
    private Servo IntakeElbowRight;
    private Servo IntakeElbowLeft;
    private boolean IntakeClawOpen = true;
    private boolean IntakeSliderChanged = false;
    final double IntakeClawPositionClose = 0.0;
    final double IntakeClawPositionOpen = 1.00;
    public class Outtake{

    }
//    public static double x_initial, x_park;
    public static double SliderVelocity = 0.9;
    public static double x_initial = 8;
    public static double y_firstSpecimen = 48.5;
    public static double y_initial = 70;
    public static double y_firstYSample = 57.50;
    public static double x_firstYSample = 43.5;
    public static double x_ascent = 22.0;
    public static double y_ascent = 15.0;
    public static double x_toScore = 50.0;
    public static double y_toScore = 63.0;
    public static double x_secondYSample = 51.0;
//    public static double y_park;
    @Override


    public void runOpMode() {
//        y_firstSpecimen = 43;
//        x_initial = -8;
//        y_initial = 70;
        //PoseVelocity2d pv = new PoseVelocity2d(new Vector2d(70,70), 50);
        Pose2d initialPose = new Pose2d(x_initial, y_initial, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //drive.setDrivePowers(pv);

        //Outtake
        // Sliders Mapping and Setup
        OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");

        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");

        // RightWheel.setDirection(CRServo.Direction.FORWARD);
        // LeftWheel.setDirection(CRServo.Direction.REVERSE);
        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        // vision here that outputs position
        int visionOutputPosition = 1;


        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        // actions that need to happen on init; for instance, a claw tightening.
        TrajectoryActionBuilder driveToHighChamber = drive.actionBuilder(initialPose)
                .lineToY(y_firstSpecimen, new TranslationalVelConstraint(90));
                //.waitSeconds(1);
        /**
         * Push the first sample and line up for first yellow sample
         */
        TrajectoryActionBuilder firstSample = drive.actionBuilder(new Pose2d(x_initial, y_firstSpecimen, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-7.04, 37.15, Math.toRadians(90.00)), Math.toRadians(-89.33))
                .splineToLinearHeading(new Pose2d(x_firstYSample, y_firstYSample, Math.toRadians(270.0)), Math.toRadians(180.0), new TranslationalVelConstraint(20));

        /**
             * Place first Sample High Basket
         */
        TrajectoryActionBuilder firstSampleScore = drive.actionBuilder(new Pose2d(x_firstYSample, y_firstYSample, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(x_toScore, y_toScore, Math.toRadians(225.0)), Math.toRadians(225.0), new TranslationalVelConstraint(20));

        /**
         * Prepare to grab Second Sample
         */
        TrajectoryActionBuilder lineUpToSecondSample = drive.actionBuilder(new Pose2d(x_toScore, y_toScore, Math.toRadians(225.0)))
                .splineToLinearHeading(new Pose2d(x_secondYSample, y_firstYSample, Math.toRadians(270.00)), Math.toRadians(270.00), new TranslationalVelConstraint(20));
               // .waitSeconds(0.5);

        /**
         * Line up to high Basket
         */
        TrajectoryActionBuilder secondSampleScore = drive.actionBuilder(new Pose2d(x_secondYSample, y_firstYSample, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(x_toScore, y_toScore, Math.toRadians(225.0)), Math.toRadians(225.0), new TranslationalVelConstraint(20));                //.lineToY(60.00)
                //.waitSeconds(0.5);
        /**
         *Park
         */
        TrajectoryActionBuilder parkAscent = drive.actionBuilder(new Pose2d(x_toScore, y_toScore, Math.toRadians(225.0)))
                .splineToLinearHeading(new Pose2d(x_ascent, y_ascent, Math.toRadians(0.0)), Math.toRadians(90.0))
                .setTangent(0.0)
                .lineToX(20.0)
                .waitSeconds(0.15);


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        /**
         *Prepare Specimen to score in the HIGH CHAMBER
         */
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         *Run Trajectory
         */
        Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamber.build()
                )
        );

        /**
         *Sliders Up to HIGH CHAMBER
         * Elbow Moved to place Specimen
         * Sliders Down After Placing Specimen
         * Elbow moved back to Middle Position
         */
        OuttakeSliders(HIGH_CHAMBER, 0, 0);

        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(400);
        OuttakeSliders(-(HIGH_CHAMBER - 50), 0, 0);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         *Spline to First Yellow Sample
         */
        Actions.runBlocking(
                new SequentialAction(
                        firstSample.build()
                )
        );
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        IntakeClaw.setPosition(IntakeClawPositionOpen);
        intakeSlidersElbow(IntakeState.OUT);
        sleep(750);
        IntakeClaw.setPosition(IntakeClawPositionClose);
        sleep(400);
        OuttakeElbowMove(OuttakeElbowPositionIn);
        sleep(500);
        intakeSlidersElbow(IntakeState.IN);
        sleep(500);
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(750);
        IntakeClaw.setPosition(IntakeClawPositionOpen);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        OuttakeSliders(HIGH_BASKET, 0 , 0);
        sleep(800);



        /**
         * Score Hihg Basket first sample
         */

        Actions.runBlocking(
                new SequentialAction(
                        firstSampleScore.build()
                )
        );
        sleep(1000);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(500);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        sleep(350);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        sleep(800);
        OuttakeSliders(-(HIGH_BASKET),0,0);
        sleep(1200);

        /**
         * Prepare Claw to get Sample from the floor
         */

        //OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionIn);
        IntakeClaw.setPosition(IntakeClawPositionOpen);
        intakeSlidersElbow(IntakeState.OUT);

        /**
         * Drive to get second Sample from Floor
         */
        Actions.runBlocking(
                new SequentialAction(
                        lineUpToSecondSample.build()
                )
        );
        OuttakeSliderLeft.setPower(0);
        OuttakeSliderRight.setPower(0);
        IntakeClaw.setPosition(IntakeClawPositionClose);
        sleep(400);
        OuttakeElbowMove(OuttakeElbowPositionIn);
        sleep(500);
        intakeSlidersElbow(IntakeState.IN);
        sleep(500);
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(750);
        IntakeClaw.setPosition(IntakeClawPositionOpen);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        OuttakeSliders(HIGH_BASKET, 0 , 0);
        sleep(800);


        /**
         * Drive to align to the HIGH Basket
         */
       // OuttakeSliders(HIGH_CHAMBER,0,0);
        Actions.runBlocking(
                new SequentialAction(
                        secondSampleScore.build()
                )
        );
        /**
         * Place second sample on the HIGH Basket
         */
        sleep(1000);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(500);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        sleep(350);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        sleep(800);
        OuttakeSliders(-(HIGH_BASKET),0,0);
        sleep(2500);


        /**
         * Park in ascent zone
         */
        Actions.runBlocking(
                new SequentialAction(
                        parkAscent.build()
                )
        );

        requestOpModeStop();


    }

    private void OuttakeSliders(int targetPosition, int velocity, int timeout) {
        // Set run modes for both motors
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        //Run to target position
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        OuttakeSliderRight.setPower(SliderVelocity);
        OuttakeSliderLeft.setPower(SliderVelocity);
    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                sleep(500);
                IntakeClaw.setPosition(0.4);
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);

                break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                break;

        }

    }

}