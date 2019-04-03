// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.control.motion;

import frc.robot.Constants;
import frc.subsystem.Drive;
import frc.subsystem.Drive.AutoDriveSignal;
import frc.subsystem.Drive.DriveSignal;
import frc.utility.OrangeUtility;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path.DrivingData;
import frc.utility.math.RigidTransform2D;
import frc.utility.math.Translation2D;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to
	 * robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private RateLimiter speedProfiler;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		speedProfiler = new RateLimiter(100, 1000); //100, 10000
		if (robotPath.isEmpty()) {
		}
	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the
	 * robot.
	 *
	 * @param robotPose
	 *            Robot position and gyro angle.
	 * @return Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(RigidTransform2D robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform2D(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		double lookAheadDist = OrangeUtility.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
				Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, lookAheadDist);
		if (data.remainingDist == 0.0) { // If robot passes point, remaining
											// distance is 0
			return new AutoDriveSignal(new DriveSignal(0, 0), true);
		}
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);
		if (robotSpeed < 20) {
			robotSpeed = 20;
		}
		Translation2D robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		//System.out.println("desired point " + robotToLookAhead.getX() + ", " + robotToLookAhead.getY());
		double radius;
		radius = getRadius(robotToLookAhead);
		double delta = (robotSpeed / radius);
		double deltaSpeed = Constants.TrackRadius * delta;

		/*
		JSONObject message = new JSONObject();
		JSONArray pose = new JSONArray();
		JSONArray lookAhead = new JSONArray();
		JSONArray closest = new JSONArray();

		if (Constants.LOGGING) {
			closest.add(data.closestPoint.getX());
			closest.add(data.closestPoint.getY());
			lookAhead.add(data.lookAheadPoint.getX());
			lookAhead.add(data.lookAheadPoint.getY());
			pose.add(robotPose.translationMat.getX());
			pose.add(robotPose.translationMat.getY());
			message.put("closest", closest);
			message.put("lookAhead", lookAhead);
			message.put("pose", pose);
			UDP.getInstance().send("10.34.76.5", message.toJSONString(), 5801);
		}*/

		if (isReversed) {
			robotSpeed *= -1;
		}
		double maxSpeed = Math.abs(robotSpeed) + Math.abs(deltaSpeed);
		if (maxSpeed > Constants.MaxPathSpeed) {
			robotSpeed -= Math.copySign(maxSpeed - Constants.MaxPathSpeed, robotSpeed);
		}
		return new AutoDriveSignal(new DriveSignal(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed), false);
	}

	private double getRadius(Translation2D robotToLookAheadPoint) {
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getY());
		return radius;
	}

	private Translation2D getRobotToLookAheadPoint(RigidTransform2D robotPose, Translation2D lookAheadPoint) {
		Translation2D lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
	}

	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime() {
		// TODO: Big Bang
		speedProfiler.reset();
	}

}
