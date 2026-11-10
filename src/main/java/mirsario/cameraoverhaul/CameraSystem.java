// Copyright 2020-2026 Mirsario & Contributors.
// Released under the GNU General Public License 3.0.
// See LICENSE.md for details.

package mirsario.cameraoverhaul;

import mirsario.cameraoverhaul.configuration.*;
import mirsario.cameraoverhaul.utilities.*;
import org.joml.SimplexNoise;
import org.joml.Vector3d;
import java.io.*;
import java.nio.file.*;
import java.util.UUID;

@SuppressWarnings("unused")
public final class CameraSystem {
	private ConfigData cfg;
	private ConfigData.Contextual ctxCfg;
	private final Vector3d prevCameraEulerRot = new Vector3d();
	private final Vector3d prevEntityVelocity = new Vector3d();
	private double lastActionTime;
	private CameraContext.Perspective prevCameraPerspective;
	private final Transform offsetTransform = new Transform();

	public void notifyOfPlayerAction() {
		lastActionTime = TimeSystem.getTime();
	}

	public void onCameraUpdate(CameraContext context, double deltaTime) {
		var time = TimeSystem.getTime();
		cfg = Configuration.get();

		updateContext(context, deltaTime);

		// Reset the offset transform
		offsetTransform.position = new Vector3d(0, 0, 0);
		offsetTransform.eulerRot = new Vector3d(0, 0, 0);

		if (!cfg.general.enabled
		|| (!cfg.general.enableInThirdPerson && context.perspective != CameraContext.Perspective.FIRST_PERSON)) {
			return;
		}

		ScreenShakes.onCameraUpdate(context, deltaTime);

		if (!context.velocity.equals(prevEntityVelocity)
		|| !context.transform.eulerRot.equals(prevCameraEulerRot)) {
			notifyOfPlayerAction();
		}

		// XY
		cameraSmoothingOffset(context, offsetTransform, deltaTime);
		noiseOffset(context, offsetTransform, deltaTime);
		// X
		verticalVelocityPitchOffset(context, offsetTransform, deltaTime);
		forwardVelocityPitchOffset(context, offsetTransform, deltaTime);
		// Z
		turningRollOffset(context, offsetTransform, deltaTime);
		strafingRollOffset(context, offsetTransform, deltaTime);

		prevEntityVelocity.set(context.velocity);
		prevCameraEulerRot.set(context.transform.eulerRot);
		prevCameraPerspective = context.perspective;
	}

	void updateContext(CameraContext context, double deltaTime) {
		if (ctxCfg == null) ctxCfg = cfg.walking.clone();

		ConfigData.Contextual target;
		if (context.isRidingVehicle) target = cfg.vehicles;
		else if (context.isRidingMount) target = cfg.mounts;
		else if (context.isSwimming) target = cfg.swimming;
		else if (context.isFlying) target = cfg.flying;
		else if (context.isSprinting) target = cfg.sprinting;
		else target = cfg.walking;

		double smoothing = cfg.general.contextTransitionSmoothing > 0 ? MathUtils.dampStep(cfg.general.contextTransitionSmoothing, deltaTime) : 1;
		ctxCfg.lerp(ctxCfg, target, MathUtils.dampStep(smoothing, deltaTime));
	}

	public void modifyCameraTransform(Transform transform) {
		transform.position.add(offsetTransform.position);
		transform.eulerRot.add(offsetTransform.eulerRot);

		ScreenShakes.modifyCameraTransform(transform);
	}

	private static final double BASE_VERTICAL_PITCH_SMOOTHING = 0.00004;
	private static final double VERTICAL_PITCH_THRESHOLD = 0.4;
	private double prevVerticalVelocityPitchOffset;

	private void verticalVelocityPitchOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		double multiplier = ctxCfg.verticalVelocityPitchFactor;
		double smoothing = BASE_VERTICAL_PITCH_SMOOTHING * ctxCfg.verticalVelocitySmoothingFactor;

		double targetOffset = context.velocity.y * multiplier;
		// Apply threshold on target, in order to fix slime block shake.
		if (Math.abs(targetOffset) < VERTICAL_PITCH_THRESHOLD) { targetOffset = 0; }
		double currentOffset = MathUtils.damp(prevVerticalVelocityPitchOffset, targetOffset, smoothing, deltaTime);

		outputTransform.eulerRot.x += currentOffset;
		prevVerticalVelocityPitchOffset = currentOffset;
	}

	private static final double BASE_FORWARD_PITCH_SMOOTHING = 0.008;
	private double prevForwardVelocityPitchOffset;

	private void forwardVelocityPitchOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		double multiplier = ctxCfg.forwardVelocityPitchFactor;
		double smoothing = BASE_FORWARD_PITCH_SMOOTHING * ctxCfg.horizontalVelocitySmoothingFactor;

		double targetOffset = context.getForwardRelativeVelocity().z * multiplier;
		double currentOffset = MathUtils.damp(prevForwardVelocityPitchOffset, targetOffset, smoothing, deltaTime);

		outputTransform.eulerRot.x += currentOffset;
		prevForwardVelocityPitchOffset = currentOffset;
	}

	private static final double BASE_TURNING_ROLL_ACCUMULATION = 0.0048;
	private static final double BASE_TURNING_ROLL_INTENSITY = 1.25;
	private static final double BASE_TURNING_ROLL_SMOOTHING = 0.0825;
	private double turningRollTargetOffset;

	// === Diagnostic trial fields ===
	// Build identity — change when diagnostics are revised under the same trial
	private static final String BUILD_REVISION = "r3";
	private static final String TRIAL_ID = "co-yaw-trial-20261109-r1";
	private static final String SOURCE_COMMIT = "51bae6f73d4b5efe9e27ed64df5445828e71a6b6";
	private static final String CANDIDATE_MODE = "normalized-yaw-step";
	private static final String CANDIDATE_VERSION = TRIAL_ID + "-" + BUILD_REVISION;
	private static final long MAX_DIAG_BYTES = 2L * 1024L * 1024L; // hard cap on file bytes
	private static final long DIAG_FLUSH_INTERVAL_MS = 2000L;
	private static final double DIAG_SAMPLE_INTERVAL = 0.1; // 10 Hz
	private static final int DIAG_TERMINAL_RESERVE = 256; // bytes reserved for terminal record

	private StringBuilder diagBuf;
	private Path diagPath;
	private long diagBytes;
	private long diagLastFlushMs;
	private long diagSeq;
	private double diagLastSampleTime;
	private String diagSessionId;
	private boolean diagEnabled;
	private boolean diagFailed;
	private boolean diagTruncated;
	private boolean diagTerminalWritten;

	private void diagInit() {
		if (diagEnabled || diagFailed || diagTruncated) return;
		diagSessionId = UUID.randomUUID().toString().substring(0, 8);
		try {
			// Resolve game logs directory via FabricLoader
			Class<?> fcl = Class.forName("net.fabricmc.loader.api.FabricLoader");
			Object inst = fcl.getMethod("getInstance").invoke(null);
			Path gameDir = (Path) fcl.getMethod("getGameDir").invoke(inst);
			diagPath = gameDir.resolve("logs").resolve("cameraoverhaul-yaw-trial-" + diagSessionId + ".csv");
		} catch (Exception e) {
			CameraOverhaul.LOGGER.error("Trial diagnostic: cannot resolve game logs dir, disabled: {}", e.getMessage());
			diagFailed = true;
			return;
		}
		try {
			Files.createDirectories(diagPath.getParent());
		} catch (Exception e) {
			CameraOverhaul.LOGGER.error("Trial diagnostic: cannot create logs dir, disabled: {}", e.getMessage());
			diagFailed = true;
			return;
		}
		try {
			diagBuf = new StringBuilder();
			diagBuf.append("schema,1.0\n");
			diagBuf.append("trial_id,").append(TRIAL_ID).append("\n");
			diagBuf.append("source_commit,").append(SOURCE_COMMIT).append("\n");
			diagBuf.append("candidate_mode,").append(CANDIDATE_MODE).append("\n");
			diagBuf.append("candidate_version,").append(CANDIDATE_VERSION).append("\n");
			diagBuf.append("session_id,").append(diagSessionId).append("\n");
			diagBuf.append("# Columns: seq,time_ms,prevPersp,currPersp,prevYaw,currYaw,rawDiff,normDiff,guardStatus,actualDiff,accumBefore,accumAfter,rollContrib,effectiveAccum,effectiveIntensity,effectiveDamping,deltaTime,wrapFlag\n");
			diagBytes = diagBuf.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8).length;
			// Write header
			Files.write(diagPath, diagBuf.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
			diagBuf = new StringBuilder();
			diagLastSampleTime = TimeSystem.getTime();
			diagLastFlushMs = System.currentTimeMillis();
			diagEnabled = true;
			// Register JVM shutdown hook for terminal record
			Runtime.getRuntime().addShutdownHook(new Thread(() -> {
				diagEnabled = true; // allow terminal write
				diagTryTerminate();
			}));
		} catch (Exception e) {
			CameraOverhaul.LOGGER.error("Failed to init trial diagnostic: {}", e.getMessage());
			diagFailed = true;
		}
	}

	private void diagFlushNow() {
		if (diagBuf == null || diagBuf.length() == 0) return;
		try {
			Files.write(diagPath, diagBuf.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
			diagBuf = new StringBuilder();
			diagLastFlushMs = System.currentTimeMillis();
		} catch (IOException e) {
			CameraOverhaul.LOGGER.error("Trial diagnostic flush failed: {}", e.getMessage());
			diagFailed = true;
			diagEnabled = false;
		}
	}

	private void diagWrite(String line) {
		if (!diagEnabled || diagFailed) return;
		// Pre-check: reserve space for terminal record
		long lineBytes = line.getBytes(java.nio.charset.StandardCharsets.UTF_8).length + 1;
		if (diagBytes + lineBytes + DIAG_TERMINAL_RESERVE > MAX_DIAG_BYTES) {
			diagBuf.append("#TRUNCATED,diag_bytes=").append(diagBytes).append("\n");
			diagFlushNow();
			CameraOverhaul.LOGGER.warn("Trial diagnostic capped at 2 MiB");
			diagTruncated = true;
			diagEnabled = false;
			return;
		}
		diagBuf.append(line).append("\n");
		diagBytes += lineBytes;
	}

	private void diagTryTerminate() {
		if (diagTerminalWritten) return;
		diagTerminalWritten = true;
		if (diagBuf == null) return;
		if (diagTruncated) {
			diagBuf.append("#TRUNCATED_END,diag_bytes=").append(diagBytes).append("\n");
		} else if (diagFailed) {
			diagBuf.append("#END,failed_earlier\n");
		} else {
			diagBuf.append("#END,session_normal\n");
		}
		diagFlushNow();
	}

	private void turningRollOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		double decaySmoothing = BASE_TURNING_ROLL_SMOOTHING * cfg.general.turningRollSmoothing;
		double intensity = BASE_TURNING_ROLL_INTENSITY * cfg.general.turningRollIntensity;
		double accumulation = BASE_TURNING_ROLL_ACCUMULATION * cfg.general.turningRollAccumulation;
		double rawYawDelta = prevCameraEulerRot.y - context.transform.eulerRot.y;
		double normDiff = MathUtils.unwrapStep(rawYawDelta);

		// Don't spazz out when switching perspectives.
		boolean guardActive = context.perspective != prevCameraPerspective;
		double actualDiff = guardActive ? 0.0 : normDiff;

		// Decay
		double accumBefore = turningRollTargetOffset;
		turningRollTargetOffset = MathUtils.damp(turningRollTargetOffset, 0, decaySmoothing, deltaTime);
		// Accumulation
		turningRollTargetOffset = MathUtils.clamp(turningRollTargetOffset + (actualDiff * accumulation), -1.0, 1.0);
		double accumAfter = turningRollTargetOffset;
		// Apply
		var turningRollContrib = MathUtils.clamp01(turningEasing(Math.abs(turningRollTargetOffset))) * intensity * Math.signum(turningRollTargetOffset);
		outputTransform.eulerRot.z += turningRollContrib;

		// Diagnostic recording — time-based 10 Hz sampling
		diagInit();
		diagSeq++;
		if (!diagEnabled) {
			diagTryTerminate();
			return;
		}
		double nowT = TimeSystem.getTime();
		boolean isWrapEvent = Math.abs(rawYawDelta) >= 180.0;
		boolean isPerspectiveChange = guardActive;
		boolean periodicDue = (nowT - diagLastSampleTime) >= DIAG_SAMPLE_INTERVAL;
		// Retain all wrap-sized events and perspective transitions; sample periodically otherwise
		if (isWrapEvent || isPerspectiveChange || periodicDue) {
			diagLastSampleTime = nowT;
			String line = diagSeq + ","
				+ String.format("%.0f", nowT * 1000.0) + ","
				+ prevCameraPerspective + ","
				+ context.perspective + ","
				+ String.format("%.4f", prevCameraEulerRot.y) + ","
				+ String.format("%.4f", context.transform.eulerRot.y) + ","
				+ String.format("%.6f", rawYawDelta) + ","
				+ String.format("%.6f", normDiff) + ","
				+ (guardActive ? "1" : "0") + ","
				+ String.format("%.6f", actualDiff) + ","
				+ String.format("%.6f", accumBefore) + ","
				+ String.format("%.6f", accumAfter) + ","
				+ String.format("%.6f", turningRollContrib) + ","
				+ String.format("%.6f", accumulation) + ","
				+ String.format("%.6f", intensity) + ","
				+ String.format("%.6f", decaySmoothing) + ","
				+ String.format("%.6f", deltaTime) + ","
				+ (isWrapEvent ? "1" : "0");
			diagWrite(line);
			long msNow = System.currentTimeMillis();
			if (msNow - diagLastFlushMs > DIAG_FLUSH_INTERVAL_MS) {
				diagFlushNow();
			}
		}
	}

	private static double turningEasing(double x) {
		// https://easings.net/#easeInOutCubic
		return x < 0.5 ? (4 * x * x * x) : (1 - Math.pow(-2 * x + 2, 3) / 2);
	}

	private static final double BASE_STRAFING_ROLL_SMOOTHING = 0.008;
	private double prevStrafingRollOffset;

	private void strafingRollOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		double multiplier = ctxCfg.strafingRollFactor;
		double smoothing = BASE_STRAFING_ROLL_SMOOTHING * ctxCfg.horizontalVelocitySmoothingFactor;

		double target = -context.getForwardRelativeVelocity().x * multiplier;
		double offset = MathUtils.damp(prevStrafingRollOffset, target, smoothing, deltaTime);

		outputTransform.eulerRot.z += offset;
		prevStrafingRollOffset = offset;
	}

	private static final double CAMERASWAY_FADING_SMOOTHNESS = 3.0;
	private double cameraSwayFactor;
	private double cameraSwayFactorTarget;

	private void noiseOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		double time = TimeSystem.getTime();
		float noiseX = (float)(time * cfg.general.cameraSwayFrequency);

		// Fade out if the player turns, moves, or does an interaction.
		if ((time - lastActionTime) < cfg.general.cameraSwayFadeInDelay) {
			cameraSwayFactorTarget = 0; // Fade-out
		}
		// Only start a fade-in after the last fade-out has ended.
		else if (cameraSwayFactor == cameraSwayFactorTarget) {
			cameraSwayFactorTarget = 1; // Fade-in
		}

		var cameraSwayFactorFadeLength = cameraSwayFactorTarget > 0 ? cfg.general.cameraSwayFadeInLength : cfg.general.cameraSwayFadeOutLength;
		var cameraSwayFactorFadeStep = cameraSwayFactorFadeLength > 0.0 ? deltaTime / cameraSwayFactorFadeLength : 1.0;
		cameraSwayFactor = MathUtils.stepTowards(cameraSwayFactor, cameraSwayFactorTarget, cameraSwayFactorFadeStep);

		var scaledIntensity =
			cfg.general.cameraSwayIntensity *
			Math.pow(cameraSwayFactor, CAMERASWAY_FADING_SMOOTHNESS);
		var target = new Vector3d(scaledIntensity, scaledIntensity, 0.0);
		var noise = new Vector3d(
			SimplexNoise.noise(noiseX, 420),
			SimplexNoise.noise(noiseX, 1337),
			SimplexNoise.noise(noiseX, 6969)
		);

		outputTransform.eulerRot.add(noise.mul(target));
	}

	private boolean msInit = false;
	private double prevYawNorm, prevPitchNorm; // last frame's normalized (vanilla) angles
	private double contYaw, contPitch; // continuous, unwrapped stream
	private double smYaw, smPitch; // actual angle

	private static final double BASE_MOUSE_SMOOTHING = 16.0;
	private static final double MOUSE_SMOOTHING_THRESHOLD = 0.001;

	private void cameraSmoothingOffset(CameraContext context, Transform outputTransform, double deltaTime) {
		final double cameraSmoothingValue = Math.max(0.0, ctxCfg.cameraSmoothing);

		final double yawNow = context.transform.eulerRot.y;
		final double pitchNow = context.transform.eulerRot.x;

		// Don't spazz out
		if (!msInit || context.perspective != prevCameraPerspective) {
			prevYawNorm = yawNow;
			prevPitchNorm = pitchNow;
			contYaw = yawNow;
			contPitch = pitchNow;
			smYaw = yawNow;
			smPitch = pitchNow;
			msInit = true;
			return;
		}

		// Preserve real mouse movement
		final double stepYaw = MathUtils.unwrapStep(yawNow - prevYawNorm);
		final double stepPitch = MathUtils.unwrapStep(pitchNow - prevPitchNorm);
		prevYawNorm = yawNow;
		prevPitchNorm = pitchNow;

		contYaw += stepYaw;
		contPitch += stepPitch;

		if (cameraSmoothingValue <= MOUSE_SMOOTHING_THRESHOLD) {
			smYaw = contYaw;
			smPitch = contPitch;
			return;
		}

		final double k = BASE_MOUSE_SMOOTHING / cameraSmoothingValue;
		final double step = 1.0 - Math.exp(-k * Math.max(0.0, deltaTime));

		final double dYaw = contYaw - smYaw;
		final double dPitch = contPitch - smPitch;

		smYaw += dYaw * step;
		smPitch += dPitch * step;

		// Apply an offset so base + offset == smoothed angles this frame
		final double yawOffset = MathUtils.wrapDiff(smYaw - yawNow);
		final double pitchOffset = MathUtils.wrapDiff(smPitch - pitchNow);

		outputTransform.eulerRot.y += yawOffset;
		outputTransform.eulerRot.x += pitchOffset;
	}
}
