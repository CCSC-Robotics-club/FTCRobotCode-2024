package org.firstinspires.ftc.teamcode.Services;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.RobotModule;
import org.firstinspires.ftc.teamcode.Utils.RobotService;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class TelemetrySender extends RobotService{
    private final Lock lock = new ReentrantLock();
    private Telemetry telemetry;
    private List<RobotModule> robotModules;
    private List<RobotService> robotServices;
    private Map<String, Object> systemMessages;
    public TelemetrySender(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addRobotModule(RobotModule robotModule) {
        this.robotModules.add(robotModule);
    }

    public void addRobotService(RobotService robotService) {
        this.robotServices.add(robotService);
    }

    public void putSystemMessage(String caption, Object object) {
        lock.lock();
        systemMessages.put(caption, object);
        lock.unlock();
    }
    public void deleteSystemMessage(String caption) {
        lock.lock();
        systemMessages.remove(caption);
        lock.unlock();
    }

    @Override
    public void init() {
        reset();
        telemetry.addLine("startup complete...");
    }

    @Override
    public void periodic(double dt) {
        telemetry.clearAll();

        for (RobotModule robotModule:robotModules) {
            telemetry.addLine("<<module:" + robotModule.moduleName + ">>");
            Map<String, Object> debugMessages = robotModule.getDebugMessages(); // TODO write a class called "message" to replace map
            for (String messageCaption:debugMessages.keySet())
                telemetry.addData(messageCaption, debugMessages.get(messageCaption));
            telemetry.addLine();
        }

        telemetry.addLine("<<services>>");
        for (RobotService robotService:robotServices) {
            if (robotService.getDebugMessages()==null || robotService.getDebugMessages().isEmpty())
                continue;

            Map<String, Object> debugMessages = robotService.getDebugMessages();
            for (String messageCaption: debugMessages.keySet())
                telemetry.addData(messageCaption, debugMessages.get(messageCaption));
            telemetry.addLine();
        }

        lock.lock();
        telemetry.addLine("\nsystem messages");
        for (String messageCaption: systemMessages.keySet())
            telemetry.addData(messageCaption, systemMessages.get(messageCaption));
        lock.unlock();

        telemetry.update();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        robotModules = new ArrayList<>(1);
        robotServices = new ArrayList<>(1);
        this.systemMessages = new HashMap<>(1);
    }

    public Telemetry getTelemetryLegacy() {
        return telemetry;
    }
}
