package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.teamcode.Modules.Chassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Rotation2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.SpeedCurves;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.BezierCurve;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Scanner;

public class SequentialCommandFactory {
    private final Chassis chassis;
    private final PositionEstimator positionEstimator;
    private Vector2D robotStartingPosition;
    private Rotation2D robotStartingRotation2D;
    private final Robot.Side side;
    private final HardwareMap hardwareMap;

    public SequentialCommandFactory(Chassis chassis, PositionEstimator positionEstimator, Robot.Side side, HardwareMap hardwareMap) {
        this(chassis, positionEstimator, new Vector2D(), new Rotation2D(0), side, hardwareMap);
    }

    public SequentialCommandFactory(Chassis chassis, PositionEstimator positionEstimator, String firstPathName, Rotation2D robotStartingRotation2D, Robot.Side side, HardwareMap hardwareMap) {
        this(chassis, positionEstimator, new Vector2D(), robotStartingRotation2D, side, hardwareMap);
        robotStartingPosition = getRobotStartingPosition(firstPathName);
        this.robotStartingRotation2D = toActualRotation(robotStartingRotation2D);
    }

    public SequentialCommandFactory(Chassis chassis, PositionEstimator positionEstimator, Vector2D robotStartingPosition, Rotation2D robotStartingRotation2D, Robot.Side side, HardwareMap hardwareMap) {
        this.chassis = chassis;
        this.positionEstimator = positionEstimator;
        this.maintainCurrentRotation = positionEstimator::getRotation2D;
        this.robotStartingPosition = robotStartingPosition;
        this.robotStartingRotation2D = robotStartingRotation2D;
        this.side = side;
        this.hardwareMap = hardwareMap;
    }

    private static final SequentialCommandSegment.InitiateCondition justGo = () -> true;
    private static final Runnable doNothing = () -> {};
    private static final SequentialCommandSegment.IsCompleteChecker weDoNotCareAboutIsItComplete = () -> true;
    private final SequentialCommandSegment.RotationFeeder maintainCurrentRotation;
    private static final SequentialCommandSegment.RotationFeeder weDoNotCareAboutRotation = () -> null;

    public SequentialCommandSegment calibratePositionEstimator() {
        return justDoIt(() -> {
            positionEstimator.setRotation(robotStartingRotation2D.getRadian());
            positionEstimator.setCurrentPosition(robotStartingPosition);
        });
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination) {
        return moveToPoint(destination, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D endingRotation) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                maintainCurrentRotation, () -> endingRotation
        );
    }

    public SequentialCommandSegment moveToPointAndStop(Vector2D destination) {
        return moveToPointAndStopIf(justGo, destination);
    }
    public SequentialCommandSegment moveToPointAndStop(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointAndStopIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination) {
        return moveToPointAndStopIf(initiateCondition, destination, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPoint(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPoint(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStop(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStop(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }


    public SequentialCommandSegment faceDirection(Rotation2D direction){
        return faceDirection(direction, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment faceDirection(Rotation2D direction, Runnable beginning, Runnable periodic, Runnable ending) {

        return new SequentialCommandSegment(
                justGo,
                () -> null,
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                positionEstimator::getRotation2D,
                () -> direction
        );
    }

    public SequentialCommandSegment lockChassis() {
        return lockChassisFor(99999);
    }
    public SequentialCommandSegment lockChassisFor(long timeMillis) {
        final long startTime = System.currentTimeMillis();
        return lockChassisIfAndUntil(justGo, () -> System.currentTimeMillis() - startTime > timeMillis);
    }
    public SequentialCommandSegment lockChassisIfAndUntil(SequentialCommandSegment.InitiateCondition initiateCondition, SequentialCommandSegment.IsCompleteChecker isCompleteChecker) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getCurrentPosition(), positionEstimator.getCurrentPosition()),
                doNothing,
                doNothing,
                doNothing,
                isCompleteChecker,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public SequentialCommandSegment justDoIt(Runnable job) {
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                job,
                doNothing,
                doNothing,
                weDoNotCareAboutIsItComplete,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public SequentialCommandSegment waitFor(long timeMillis) {
        final long[] t0 = new long[1];
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                () -> t0[0] = System.currentTimeMillis(),
                doNothing, doNothing,
                () -> System.currentTimeMillis() - t0[0] > timeMillis,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public Vector2D getRobotStartingPosition(String firstPathName) {
        return getBezierCurvesFromPathFile(firstPathName).get(0).getPositionWithLERP(0);
    }

    public SequentialCommandSegment followSingleCurve(String pathName, int index, Rotation2D facingRotation) {
        return followSingleCurve(pathName, index, facingRotation, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment followSingleCurve(String pathName, int index, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending) {
        return followSingleCurve(pathName, index, facingRotation, beginning, periodic, ending, SpeedCurves.originalSpeed, 1);
    }

    public SequentialCommandSegment followSingleCurve(String pathName, int index, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending, SpeedCurves.SpeedCurve speedCurve, double timeScale) {
        final List<BezierCurve> curves = getBezierCurvesFromPathFile(pathName);
        return new SequentialCommandSegment(
                () -> true,
                () -> curves.get(index),
                beginning, periodic, ending,
                () -> true,
                maintainCurrentRotation, () -> toActualRotation(facingRotation),
                speedCurve, timeScale
        );
    }

    public SequentialCommandSegment followSingleCurveAndStop(String pathName, int index, Rotation2D facingRotation) {
        return followSingleCurveAndStop(pathName, index, facingRotation, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment followSingleCurveAndStop(String pathName, int index, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending) {
        final List<BezierCurve> curves = getBezierCurvesFromPathFile(pathName);
        return new SequentialCommandSegment(
                () -> true,
                () -> curves.get(index),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                maintainCurrentRotation, () -> toActualRotation(facingRotation),
                SpeedCurves.originalSpeed,1
        );
    }

    /** all rotations are in red alliance, will be automatically converted if blue */
    public List<SequentialCommandSegment> followPathFacing(String pathName, Rotation2D facingRotation) {
        return followPathFacing(pathName, facingRotation, doNothing, doNothing, doNothing);
    }
    public List<SequentialCommandSegment> followPathFacing(String pathName, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending) {
        final Rotation2D[] rotationTargets = new Rotation2D[getBezierCurvesFromPathFile(pathName).size()];
        Arrays.fill(rotationTargets, facingRotation);
        return followPath(pathName, rotationTargets, beginning, periodic, ending);
    }

    public List<SequentialCommandSegment> followPath(String pathName) {
        return followPath(pathName, doNothing, doNothing, doNothing);
    }


    public List<SequentialCommandSegment> followPath(String pathName, Runnable beginning, Runnable periodic, Runnable ending) {
        return followPath(pathName, new Rotation2D[getBezierCurvesFromPathFile(pathName).size()+1], beginning, periodic, ending);
    }

    public List<SequentialCommandSegment> followPath(String pathName, Rotation2D[] robotRotationTargets, Runnable beginning, Runnable periodic, Runnable ending) {
        final List<BezierCurve> curves = getBezierCurvesFromPathFile(pathName);
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        if (curves.size() != robotRotationTargets.length)
            throw new IllegalStateException("Error While Scheduling Follow Path Command: " + pathName + ". Rotational targets length (" + robotRotationTargets.length + ") do not match pathplanner checkpoints number (" + curves.size() + ")");

        if (curves.size() == 1)
            return Collections.singletonList(new SequentialCommandSegment(
                    () -> true,
                    () -> curves.get(0),
                    beginning, periodic, ending,
                    chassis::isCurrentTranslationalTaskRoughlyComplete,
                    positionEstimator::getRotation2D, () -> toActualRotation(robotRotationTargets[0]),
                    SpeedCurves.easeInOut, 1
            ));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> curves.get(0),
                beginning, periodic, doNothing,
                () -> true,
                maintainCurrentRotation, () -> toActualRotation(robotRotationTargets[0]),
                SpeedCurves.easeIn,1
        ));

        for (int i = 1; i < curves.size()-1; i++) {
            final BezierCurve curve = curves.get(i);
            final Rotation2D startingRotation = robotRotationTargets[i-1], endingRotation = robotRotationTargets[i];
            commandSegments.add(new SequentialCommandSegment(
                    () -> true,
                    () -> curve,
                    doNothing, periodic, doNothing,
                    () -> true,
                    () -> toActualRotation(startingRotation), () -> toActualRotation(endingRotation),
                    SpeedCurves.originalSpeed,1
            ));
        }

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> curves.get(curves.size()-1),
                doNothing, periodic, ending,
                chassis::isCurrentTranslationalTaskRoughlyComplete,
                () -> toActualRotation(robotRotationTargets[robotRotationTargets.length-2]), () -> toActualRotation(robotRotationTargets[robotRotationTargets.length-1]),
                SpeedCurves.originalSpeed,1
        ));

        return commandSegments;
    }

    public List<BezierCurve> getBezierCurvesFromPathFile(String pathName) {
        try {
            InputStream is = hardwareMap.appContext.getAssets().open("deploy/pathplanner/paths/" + pathName + ".path");
            Scanner scanner = new Scanner(is).useDelimiter("\\A");
            String jsonContent = scanner.hasNext() ? scanner.next() : "";

            JSONObject pathJson = new JSONObject(jsonContent);
            JSONArray waypointsJson = pathJson.getJSONArray("waypoints");

            List<BezierCurve> curves = new ArrayList<>();
            for (int i = 0; i < waypointsJson.length() - 1; i++) {
                JSONObject point = (JSONObject) waypointsJson.get(i),
                        nextPoint = (JSONObject) waypointsJson.get(i+1);

                curves.add(new BezierCurve(
                        pointFromJson((JSONObject) point.get("anchor")),
                        pointFromJson((JSONObject) point.get("nextControl")),
                        pointFromJson((JSONObject) nextPoint.get("prevControl")),
                        pointFromJson((JSONObject) nextPoint.get("anchor"))
                ));
            }
            return curves;
        } catch (IOException e) {
            throw new RuntimeException("error while reading json file");
        } catch (JSONException e) {
            throw new RuntimeException("error while parsing json file");
        }
    }

    /**
     * converts a point from pathplanner to vector2D
     * pathplanner is always in red alliance
     * converts to red / blue alliance according to driver-station, defaults to red
     * */
    private Vector2D pointFromJson(JSONObject pointJson) throws JSONException {
        final double x = ((Number) pointJson.get("x")).doubleValue();
        final double y = ((Number) pointJson.get("y")).doubleValue();

        return toActualPosition(new Vector2D(new double[] {x * 100, y * 100}));
    }

    private Rotation2D toActualRotation(Rotation2D rotationOnBlueSide) {
        return side == Robot.Side.BLUE ? rotationOnBlueSide : new Rotation2D(-rotationOnBlueSide.getRadian());
    }

    private Vector2D toActualPosition(Vector2D positionOnBlueSide) {
        return side == Robot.Side.BLUE ? positionOnBlueSide :
                new Vector2D(new double[] {
                        365.76 - positionOnBlueSide.getX(), // TODO make it a constant
                        positionOnBlueSide.getY()
                });
    }
}
