package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum.ObjectDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;


public class TestingConeVis extends LinearOpMode {
    public ObjectDetectionPipeline pipeline;
    RockBotHardware robot = new RockBotHardware();
    @Override

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, true);
         pipeline = new ObjectDetectionPipeline(this.telemetry);

        ElapsedTime stopwatch = new ElapsedTime();
        double seconds = stopwatch.seconds();
        while(opModeIsActive() && pipeline.winner == -1){

        }
        robot.camera.stopStreaming();
        telemetry.addData("Winner is", pipeline.winner);
        telemetry.update();
        while(opModeIsActive()){

        }
    }
}
