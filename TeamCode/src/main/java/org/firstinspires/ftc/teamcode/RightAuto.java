package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.Crush;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@Config
@Autonomous(name = "Right Auto", group = "ITD Auto", preselectTeleOp = "AyMarthaV2")
public class RightAuto extends LinearOpMode {

    public class Sliders {
        final int HIGH_BASKET = 3600;
        final int HIGH_CHAMBER = 1000;
        private DcMotorEx OuttakeSliderRight;
        private DcMotorEx OuttakeSliderLeft;
        public Sliders(HardwareMap hardwareMap) {
            OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
            OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        }

        public class InitializeSliders implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    OuttakeSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
                    OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

                    Crush.getInstance().setInitialPositions(OuttakeSliderRight.getCurrentPosition(), OuttakeSliderLeft.getCurrentPosition());
                    initialPositionRight = Crush.getInstance().getRight();
                    initialPositionLeft = Crush.getInstance().getLeft();

                    initialized = true;
                    return true;
                }else{
                    return false;
                }


            }
        }
        public Action InitializeSliders() {
            return new InitializeSliders();
        }
        public class SlidersHighChamber implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Synchronize motion
                    OuttakeSliderRight.setTargetPosition(HIGH_CHAMBER + initialPositionRight);
                    OuttakeSliderLeft.setTargetPosition(HIGH_CHAMBER + initialPositionLeft);

                    //Run to target position
                    OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Set power
                    OuttakeSliderRight.setPower(SliderVelocity);
                    OuttakeSliderLeft.setPower(SliderVelocity);

                    initialized = true;
                }

                double posr = OuttakeSliderRight.getCurrentPosition();
                double posl = OuttakeSliderLeft.getCurrentPosition();
                packet.put("SliderPosRight", posr);
                packet.put("SliderPosLeft", posl);
                if ((posr < (HIGH_CHAMBER + initialPositionRight)) && (posl < (HIGH_CHAMBER + initialPositionLeft))) {
                    return true;
                } else {
//                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action SlidersHighChamber() {
            return new SlidersHighChamber();
        }

        public class SlidersDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Synchronize motion
                    OuttakeSliderRight.setTargetPosition(initialPositionRight);
                    OuttakeSliderLeft.setTargetPosition(initialPositionLeft);

                    //Run to target position
                    OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Set power
                    OuttakeSliderRight.setPower(SliderVelocity);
                    OuttakeSliderLeft.setPower(SliderVelocity);

                    initialized = true;
                }

                double posr = OuttakeSliderRight.getCurrentPosition();
                double posl = OuttakeSliderLeft.getCurrentPosition();
                packet.put("SliderPosRight", posr);
                packet.put("SliderPosLeft", posl);
                if (posr > initialPositionRight && posl > initialPositionLeft) {
                    return true;
                } else {
                    OuttakeSliderRight.setPower(0);
                    OuttakeSliderLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action SlidersDown(){
            return new SlidersDown();
        }
    }

    public class Claw {
        private Servo OuttakeClaw;
        final double OuttakeClawPositionClose = 1.0;
        final double OuttakeClawPositionOpen = 0.00;

        public Claw(HardwareMap hardwareMap) {
            OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
            OuttakeClaw.setDirection(Servo.Direction.FORWARD);

        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OuttakeClaw.setPosition(OuttakeClawPositionClose);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    public class OuttakeElbow {
        private Servo OuttakeElbowRight;
        private Servo OuttakeElbowLeft;
        final double OuttakeElbowPositionOut = 0.15;
        final double OuttakeElbowPositionIn = 0.85;
        final double OuttakeElbowPositionMiddle = 0.48;

        public OuttakeElbow(HardwareMap hardwareMap) {
            OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
            OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");

            OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
            OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        }

        public class OuttakeElbowMiddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OuttakeElbowLeft.setPosition(OuttakeElbowPositionMiddle);
                OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                return false;
            }
        }
        public Action OuttakeElbowMiddle() {
            return new OuttakeElbowMiddle();
        }
        public class OuttakeElbowOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                return false;
            }
        }
        public Action OuttakeElbowOut() {
            return new OuttakeElbowOut();
        }
        public class OuttakeElbowIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OuttakeElbowLeft.setPosition(OuttakeElbowPositionIn);
                OuttakeElbowRight.setPosition(OuttakeElbowPositionIn);
                return false;
            }
        }
        public Action OuttakeElbowIn() {
            return new OuttakeElbowIn();
        }
    }


    //private Servo OuttakeClaw;

    public int initialPositionLeft, initialPositionRight;


    final double OuttakeElbowPositionOutSpecimen = 0.1;
    //final double OuttakeElbowPositionMiddle = 0.55;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private enum IntakeState{
        IN,
        OUT
    }
    final double IntakeSliderPositionOut = 0.60;
    final double IntakeSliderPositionIN = 0.0;

//    public static double x_initial, x_park;
    public static double SliderVelocity = 1.0;
    public static double x_initial = -8;
    public static double y_firstSpecimen = 52;
    public static double y_firstWallSpecimen = 62.5;
    public static double x_firstWallSpecimen = -46.0;
    public static double x_secondWallSpecimen = -42.0;
    public static double y_secondWallSpecimen = 62.0;
    public static double y_thirdSpecimen = 44.0;
    public static double y_secondSpecimen = 46.0;
    public static double x_secondSpecimen = -8.0;
    public static double y_initial = 70;
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
        Sliders sliders = new Sliders(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        OuttakeElbow elbow = new OuttakeElbow(hardwareMap);

        //Outtake
        // Sliders Mapping and Setup
        if(!Crush.getInstance().areSlidersInitialized()){
            Actions.runBlocking(sliders.InitializeSliders());
        }


        //Servo Claw, Elbow, and Wrist Mapping and Setup

        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");


        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());

        TrajectoryActionBuilder driveToHighChamber = drive.actionBuilder(initialPose)
                .lineToY(y_firstSpecimen, new TranslationalVelConstraint(90));
                //.waitSeconds(1);
        /**
         * Push the first sample and line up for second sample
         */
        TrajectoryActionBuilder firstSample = drive.actionBuilder(new Pose2d(x_initial, y_firstSpecimen, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-7.04, 37.15, Math.toRadians(90.00)), Math.toRadians(-89.33))
                .splineToLinearHeading(new Pose2d(-33.0, 40.0, Math.toRadians(270.0)), Math.toRadians(270.0))
                //.setTangent(270.0)
                .lineToY(20.0, new TranslationalVelConstraint(90))
                //.turnTo(270.0)
                .strafeTo(new Vector2d(-45.0,20.0), new TranslationalVelConstraint(90))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(58.0, new TranslationalVelConstraint(90))
                //.waitSeconds(1.5)
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(23.0, new TranslationalVelConstraint(90))
                .strafeTo(new Vector2d(-58.0,23.0), new TranslationalVelConstraint(90));
                //.setTangent(Math.toRadians(270.0))

        /**
             * Push the second sample
         */
        TrajectoryActionBuilder secondSample = drive.actionBuilder(new Pose2d(-58, 18.0, Math.toRadians(270.00)))
                .lineToYConstantHeading(55.0, new TranslationalVelConstraint(90));
                //.waitSeconds(0.5);
        /**
         * Prepare to grab Specimen
         */

        TrajectoryActionBuilder lineUpToSecondSpecimen = drive.actionBuilder(new Pose2d(-58, 53.0, Math.toRadians(270.00)))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(50.0)
                //.strafeTo(new Vector2d(-40.0,50.0), 1000, 500)
                .strafeTo(new Vector2d(x_firstWallSpecimen,50.0))
                .setTangent(Math.toRadians(270.0))
                .lineToY(y_firstWallSpecimen, new TranslationalVelConstraint(70));
               // .waitSeconds(0.5);

        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder driveToHighChamberSecondSample = drive.actionBuilder(new Pose2d(x_firstWallSpecimen, y_firstWallSpecimen, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(x_secondSpecimen, y_secondSpecimen, Math.toRadians(90.0)), Math.toRadians(180.0));
                //.lineToY(60.00)
                //.waitSeconds(0.5);
/**
 *Park
 */
        TrajectoryActionBuilder parkObservationZone2 = drive.actionBuilder(new Pose2d(x_secondSpecimen, y_secondSpecimen, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(x_secondWallSpecimen, 53.0, Math.toRadians(270.0)), Math.toRadians(270.0))
                //.setTangent(270.0)
                .lineToY(y_secondWallSpecimen);
                //.waitSeconds(0.15);
        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder driveToHighChamberThirdSample = drive.actionBuilder(new Pose2d(x_secondWallSpecimen, y_secondWallSpecimen, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(-6.0, y_thirdSpecimen, Math.toRadians(90.0)), Math.toRadians(90.0));
       /* *//**
         *Park
         *//*
        TrajectoryActionBuilder parkObservationZone2 = drive.actionBuilder(new Pose2d(-3.0, y_firstSpecimen - 2.25, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(-42.0, 53.0, Math.toRadians(270.0)), Math.toRadians(270.0))
                //.setTangent(270.0)
                .lineToY(61.0)
                .waitSeconds(0.15);*/

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        /**
         *Prepare Specimen to score in the HIGH CHAMBER
         */


//        OuttakeElbowMove(OuttakeElbowPositionMiddle);

//        OuttakeSliders(HIGH_CHAMBER, 0, 0);
        /**
         *Run Trajectory
         */
        Actions.runBlocking(
                new SequentialAction(
                        elbow.OuttakeElbowMiddle(),
                        sliders.SlidersHighChamber(),
                        driveToHighChamber.build()
                )
        );

        /**
         *Sliders Up to HIGH CHAMBER
         * Elbow Moved to place Specimen
         * Sliders Down After Placing Specimen
         * Elbow moved back to Middle Position
         */
//        OuttakeSliders(HIGH_CHAMBER, 0, 0);
        /**
         *Spline to First Sample
         * Push First Sample to Observation Zone
         * Drive Back to line up for Second Sample
         */

        sleep(500);
        //OuttakeElbowMove(OuttakeElbowPositionOut);
        Actions.runBlocking(elbow.OuttakeElbowOut());
        sleep(400);
        /*Actions.runBlocking(new SequentialAction(
                                claw.openClaw(),
                                sliders.SlidersDown(),
                                elbow.OuttakeElbowMiddle()
                            )
        );*/
        //OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        //OuttakeSliders(-(HIGH_CHAMBER - 50), 0, 0);
        /*sleep(300);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);*/
        //sleep(100);
        //OuttakeElbowMove(OuttakeElbowPositionMiddle);

        /**
         *Spline to First Sample
         * Push First Sample to Observation Zone
         * Drive Back to line up for Second Sample
         */

        /**
         * Push Second Sample to Observation Zone
         */

        /**
         * Prepare Claw to get Specimen from Wall
         */

        Actions.runBlocking(
                new SequentialAction(
                        claw.openClaw(),
                        firstSample.build(),
                        sliders.SlidersDown(),
                        elbow.OuttakeElbowMiddle(),
                        secondSample.build(),
                        elbow.OuttakeElbowOut() //Prepare Claw to get Specimen from Wall


                )
        );

        /* *//**
         * Push Second Sample to Observation Zone
         *//*

        Actions.runBlocking(
                new SequentialAction(
                        secondSample.build()
                )
        );
*/
        /**
         * Prepare Claw to get Specimen from Wall
         */
        //OuttakeClaw.setPosition(OuttakeClawPositionOpen);
//        OuttakeElbowMove(OuttakeElbowPositionOut);
//        OuttakeSliderLeft.setPower(0);
//        OuttakeSliderRight.setPower(0);

        /**
         * Drive to get second Specimen from Wall
         */
        Actions.runBlocking(
                new SequentialAction(
                        lineUpToSecondSpecimen.build(),
                        claw.closeClaw()
                )
        );

        /**
         * Grab second Specimen from Wall and Prepare Elbow to Transport
         */
//        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(350);
//        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         * Drive to align to the HIGH CHAMBER
         */
//        OuttakeSliders(HIGH_CHAMBER,0,0);
        Actions.runBlocking(
                new SequentialAction(
                        elbow.OuttakeElbowMiddle(),
                        driveToHighChamberSecondSample.build(),
                        sliders.SlidersHighChamber()
                )
        );
        /**
         * Place second sample on the HIGH CHAMBER
         */

        //sleep(300);
        //OuttakeSliders(HIGH_CHAMBER,0,0);
        sleep(250);
        Actions.runBlocking(elbow.OuttakeElbowOut());
        //OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(350);
        Actions.runBlocking(new SequentialAction(
                                sliders.SlidersDown(),
                                claw.openClaw(),
                                elbow.OuttakeElbowMiddle(),
                                parkObservationZone2.build(),
                                claw.closeClaw()
                            )
        );
        /*OuttakeSliders(-(HIGH_CHAMBER-50),0,0);
        sleep(350);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionOut);*/

        // OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        /**
         * Prepare for third specimen
         */
       /* Actions.runBlocking(
                new SequentialAction(
                        parkObservationZone2.build()
                )
        );*/
        //OuttakeElbowMove(OuttakeElbowPositionOut);
        /**
         * Grab second Specimen from Wall and Prepare Elbow to Transport
         */
       // OuttakeElbowMove(OuttakeElbowPositionOutSpecimen);

//        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(350);
        Actions.runBlocking(new SequentialAction(
                            elbow.OuttakeElbowMiddle(),
                            sliders.SlidersHighChamber(),
                            driveToHighChamberThirdSample.build(),
                            elbow.OuttakeElbowOut()
                )

        );
//        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         * Drive to align to the HIGH CHAMBER
         */
//        OuttakeSliders(HIGH_CHAMBER,0,0);

        /*Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamberThirdSample.build()
                )
        );*/
        /**
         * Place second sample on the HIGH CHAMBER
         */

        //sleep(350);
//        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(350);
        Actions.runBlocking(new SequentialAction(
                                sliders.SlidersDown(),
                                claw.openClaw(),
                                elbow.OuttakeElbowMiddle()
                            )
        );
       /* OuttakeSliders(-(HIGH_CHAMBER),0,0);
        sleep(200);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        sleep(400);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);*/


       // sleep(500);


    }

/*    private void OuttakeSliders(int targetPosition, int velocity, int timeout) {
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
    }*/
 /*   private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }*/
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
               break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                break;

        }

    }

}