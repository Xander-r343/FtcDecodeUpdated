package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Configs.RedAutoPaths;
import org.firstinspires.ftc.teamcode.Sensors.OdoPods;
import org.firstinspires.ftc.teamcode.Subsystems.Aimbots;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
@Autonomous(name = "9 ball auto red")
public class FarRedAuto extends LinearOpMode {

    Servo rgb;
    ShooterSubsystem robotSubsystem;
    OdoPods pods;
    MecanumDrivetrain drivetrain;
    RedAutoPaths pathDatabase;
    Object Xpos;
    Object Ypos;
    Object Headingpos;
    Object Alliance;
    Config config;
    int AutoState;
    ElapsedTime timer;
    double speed;
    double time;
    Aimbots aimbots;
    private ElapsedTime runtime = new ElapsedTime();
    int tagID;
    int flywheelSpeed;
    double driveSpeed;
    double headingSpeed;
    double targetH;
    double nextX;
    double nextY;
    double nextH;
    double nextPower;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(1, hardwareMap);
        pods = new OdoPods(hardwareMap, drivetrain);
        pathDatabase = new RedAutoPaths();
        config = new Config();
        robotSubsystem = new ShooterSubsystem(hardwareMap);
        //intialize blackboard objects
        AutoState = 0;
        //initialize odopods by using the config class
        pods.setPosition(88, 9, 0);
        timer = new ElapsedTime();
        AutoState = 0;
        aimbots = new Aimbots(config.RedAlliance, pods, hardwareMap);
        rgb = hardwareMap.get(Servo.class, config.RBGName);
        flywheelSpeed = 2600;
        robotSubsystem.setServoPosition(0.5);
        driveSpeed = 0.4;
        waitForStart();
        headingSpeed = 0.5;
        runtime.reset();
        timer.reset();
        blackboard.put(config.AllianceKey,config.RedAlliance);
        while (opModeIsActive()) {
            aimbots.update();
            switch (AutoState) {
                case 0: {
                    //drive forward spin intake, etc
                    aimbots.startLL();
                    aimbots.switchPipeline(1);
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    while(!pods.holdPosition(88,84,0,1)){
                        pods.update();
                        telemetry.addData("x", pods.getX());
                        telemetry.addData("y", pods.getY());
                        telemetry.addData("h", pods.getHeading());
                        telemetry.addData("I see apriltag: ", aimbots.LLstatusIsValid());
                        telemetry.update();
                        if(pods.holdPosition(88,84,0,1)){
                            AutoState = 1;
                            targetH = aimbots.getIdealAngle();
                        }
                    }
                }
                break;
                case 1: {
                    //turn robot to face goal and fire 3 artifacts
                    targetH = aimbots.getIdealAngle();
                    timer.startTime();
                    while(timer.seconds() < 3.2) {
                        double targetH;
                        //align using pods to shoot in goal
                        targetH = aimbots.getIdealAngle();

                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 2.9){
                            AutoState = 2;
                        }
                    }
                }
                break;
                case 2:{
                    double speed = 0.6;
                    robotSubsystem.setServoPosition(1);
                    //fire the first 3 artifacts
                    timer.reset();
                    timer.startTime();
                    while(timer.seconds() < 4) {
                        robotSubsystem.spinIntake(speed);
                        robotSubsystem.spinBelt(0.4);
                        if(timer.seconds() > 4.8){
                            shutOff();
                            robotSubsystem.setServoPosition(0);
                            AutoState = 3;
                            robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                            robotSubsystem.spinIntake(1);
                        }
                        else if(timer.seconds() > 0.4 && timer.seconds() < 2){
                            robotSubsystem.setServoPosition(0);
                        }
                        if(timer.seconds() > 2 && timer.seconds() < 4){
                            robotSubsystem.setServoPosition(1);
                        }
                        speed -= 0.0003;
                    }
                    robotSubsystem.setServoPosition(0);
                    AutoState = 3;
                }
                break;
                case 3:{
                    pods.update();
                    //strafe to align with pickup balls
                    while(!pods.holdPosition(98,84,90,1)) {
                        pods.update();
                    }
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(0.4);
                    timer.reset();
                    timer.startTime();
                    //pickup 2 more balls from far pickup
                    while(timer.seconds() < 1.5){
                        //adjust this 115 x for 3rd ball if necessary
                        pods.holdPosition(133,84,90,1);
                        pods.update();
                    }
                    while(!pods.holdPosition(98, 95, 90, 1)) {
                        //move back to fire
                        pods.update();

                    }
                    AutoState = 4;
                    robotSubsystem.spinIntake(1);
                }
                break;
                case 4:{
                    timer.reset();
                    //align to goal using limelight
                    while(timer.seconds() < 2) {
                        double targetH;
                        targetH = aimbots.getIdealAngle();
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                    }
                    //fire balls
                    while(timer.seconds() < 7) {
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.5);
                        if(timer.seconds() > 6.9){
                            shutOff();
                            AutoState = 5;
                            robotSubsystem.setServoPosition(0);
                        }
                    }
                }
                break;
                case 5:{
                    while(timer.seconds() < 7.3) {
                        pods.holdHeading(90, 1);
                        pods.update();
                    }
                    robotSubsystem.spinIntake(1);
                    //prepare for another pickup
                    while(timer.seconds() < 8.5){
                        pods.holdPosition(96,60.5,90,1);
                        pods.update();
                    }
                    AutoState = 6;
                }
                break;
                case 6:{
                    //pickup ball
                    timer.reset();
                    timer.startTime();
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(1);
                    while(timer.seconds() < 2.2){
                        pods.holdPosition(138, 60, 90, 0.3);
                        pods.update();
                    }
                    AutoState = 7;
                }
                break;
                case 7: {
                    while (timer.seconds() < 3.8) {
                        pods.holdPosition(94, 90, 90, 1);
                        pods.update();
                    }
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    while(timer.seconds() < 5.2){
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        /*if (aimbots.LLstatusIsValid()) {
                            //targetH = pods.getHeading() - aimbots.getHeadingErrorLL();

                        } else {*/
                        targetH = aimbots.getIdealAngle();
                        //}
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 4.5){
                            AutoState = 8;
                        }
                    }
                }
                break;
                case 8:{
                    while(timer.seconds() < 7.4) {
                        robotSubsystem.spinIntake(0.4);
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.4);
                    }
                    robotSubsystem.setServoPosition(0);

                    pods.holdPosition(129, 90, 90, 1);
                    pods.update();

                }
                break;
            }

        }

        aimbots.update();
        telemetry.addData("x", pods.getX());
        telemetry.addData("y", pods.getY());
        telemetry.addData("h", pods.getHeading());
        telemetry.addData("I see apriltag: ", aimbots.LLstatusIsValid());
        telemetry.update();
    }
    public boolean goToPosition(){
        return pods.holdPosition(nextX,nextY,nextH,nextPower);
    }
    public boolean goToHeading(){
        return pods.holdHeading(nextH,nextPower);
    }





    public void resetArtifacts(){
        robotSubsystem.setFlywheelVelocity(-300);
        robotSubsystem.spinBelt(-0.2);
        robotSubsystem.spinIntake(1);
    }
    public void shutOff(){
        robotSubsystem.spinBelt(0);
    }
}
