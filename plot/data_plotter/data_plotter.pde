import processing.net.*;
import java.util.ArrayList;

final String ESP32_IP = "192.168.4.1";
final int ESP32_PORT = 23;

final float WINDOW_SECONDS = 10.0;
final float MIN_ABS_BOUND = 10.0;

float yMin = -MIN_ABS_BOUND;
float yMax = MIN_ABS_BOUND;

Client client;
long lastConnectAttemptMs = 0;
final int RECONNECT_INTERVAL_MS = 1000;

final int BG_COLOR = color(6, 10, 8);
final int PLOT_BG = color(10, 18, 12);
final int BORDER_COLOR = color(40, 90, 55);
final int GRID_COLOR = color(22, 55, 32);
final int GRID_ZERO_COLOR = color(60, 140, 80);
final int TEXT_COLOR = color(165, 225, 170);
final int TRACE_X = color(70, 255, 120);
final int TRACE_Y = color(110, 240, 170);
final int TRACE_Z = color(170, 255, 110);

class Sample {
	float t;
	float ax;
	float ay;
	float az;
	long sensorTsUs;

	Sample(float t, float ax, float ay, float az, long sensorTsUs) {
		this.t = t;
		this.ax = ax;
		this.ay = ay;
		this.az = az;
		this.sensorTsUs = sensorTsUs;
	}
}

ArrayList<Sample> samples = new ArrayList<Sample>();

// Pending values while waiting for one full XYZ sample packet.
long pendingTimestampUs = 0;
float pendingX = 0;
float pendingY = 0;
float pendingZ = 0;
boolean hasTimestamp = false;
boolean hasX = false;
boolean hasY = false;
boolean hasZ = false;

boolean sensorClockInitialized = false;
long sensorSegmentStartUs = 0;
long lastSensorTsUs = 0;
float sensorSegmentStartPlotS = 0;

ArrayList<Float> receiveEventsSec = new ArrayList<Float>();
float receivingFrequencyHz = 0;
float sendingFrequencyHz = -1;

void setup() {
	size(1920, 1080, P2D);
	pixelDensity(displayDensity());
	smooth(2);
	frameRate(60);
	textFont(createFont("Consolas", 18));
	surface.setTitle("KuDAQ - Live Acceleration Plot");
	connectIfNeeded();
}

void draw() {
	background(BG_COLOR);

	connectIfNeeded();
	readTcpStream();
	updateReceivingFrequency(millis() / 1000.0);
	pruneOldSamples();
	updateYBounds();

	drawAxes();
	drawCurves();
	drawLegendAndStatus();
}

void connectIfNeeded() {
	if (client != null && client.active()) {
		return;
	}

	if (millis() - lastConnectAttemptMs < RECONNECT_INTERVAL_MS) {
		return;
	}

	lastConnectAttemptMs = millis();
	println("Trying to connect to " + ESP32_IP + ":" + ESP32_PORT + "...");

	try {
		client = new Client(this, ESP32_IP, ESP32_PORT);
		println("Connected.");
	}
	catch (Exception e) {
		client = null;
		println("Connection failed: " + e.getMessage());
	}
}

void readTcpStream() {
	if (client == null || !client.active()) {
		return;
	}

	while (client.available() > 0) {
		String rawLine = client.readStringUntil('\n');
		if (rawLine == null) {
			break;
		}

		String line = trim(rawLine);
		if (line.length() == 0) {
			continue;
		}

		parseLine(line);
	}
}

void parseLine(String line) {
	int sep = line.indexOf(':');
	if (sep <= 0 || sep >= line.length() - 1) {
		return;
	}

	String key = line.substring(0, sep);
	String valueText = line.substring(sep + 1);

	if (key.equals("TIMESTAMP")) {
		try {
			pendingTimestampUs = Long.parseLong(valueText);
			hasTimestamp = true;
		}
		catch (Exception e) {
			return;
		}
	} else if (key.equals("ACCEL_X")) {
		try {
			pendingX = Float.parseFloat(valueText);
			hasX = true;
		}
		catch (Exception e) {
			return;
		}
	} else if (key.equals("ACCEL_Y")) {
		try {
			pendingY = Float.parseFloat(valueText);
			hasY = true;
		}
		catch (Exception e) {
			return;
		}
	} else if (key.equals("ACCEL_Z")) {
		try {
			pendingZ = Float.parseFloat(valueText);
			hasZ = true;
		}
		catch (Exception e) {
			return;
		}
	} else if (key.equals("SENDING_FREQUENCY")) {
		try {
			sendingFrequencyHz = Float.parseFloat(valueText);
		}
		catch (Exception e) {
			return;
		}
	}

	// The ESP32 now sends one full vector packet: TIMESTAMP + X + Y + Z.
	if (hasTimestamp && hasX && hasY && hasZ) {
		float t = computePlotTimeSeconds(pendingTimestampUs);
		samples.add(new Sample(t, pendingX, pendingY, pendingZ, pendingTimestampUs));

		float nowSec = millis() / 1000.0;
		receiveEventsSec.add(nowSec);
		updateReceivingFrequency(nowSec);

		hasTimestamp = false;
		hasX = false;
		hasY = false;
		hasZ = false;
	}
}

float computePlotTimeSeconds(long sensorTsUs) {
	if (!sensorClockInitialized) {
		sensorClockInitialized = true;
		sensorSegmentStartUs = sensorTsUs;
		lastSensorTsUs = sensorTsUs;
		sensorSegmentStartPlotS = 0;
		return 0;
	}

	// Handle timestamp resets/wraps while keeping the plotted time monotonic.
	if (sensorTsUs < lastSensorTsUs) {
		sensorSegmentStartPlotS = samples.isEmpty() ? 0 : samples.get(samples.size() - 1).t;
		sensorSegmentStartUs = sensorTsUs;
	}

	lastSensorTsUs = sensorTsUs;
	return sensorSegmentStartPlotS + (sensorTsUs - sensorSegmentStartUs) / 1000000.0;
}

void updateReceivingFrequency(float nowSec) {
	while (!receiveEventsSec.isEmpty() && nowSec - receiveEventsSec.get(0) > 1.0) {
		receiveEventsSec.remove(0);
	}

	receivingFrequencyHz = receiveEventsSec.size();
}

void pruneOldSamples() {
	if (samples.isEmpty()) {
		return;
	}

	float newestT = samples.get(samples.size() - 1).t;
	float cutoff = newestT - WINDOW_SECONDS;

	while (!samples.isEmpty() && samples.get(0).t < cutoff) {
		samples.remove(0);
	}
}

void updateYBounds() {
	float maxAbs = MIN_ABS_BOUND;
	for (Sample s : samples) {
		maxAbs = max(maxAbs, abs(s.ax));
		maxAbs = max(maxAbs, abs(s.ay));
		maxAbs = max(maxAbs, abs(s.az));
	}

	// Add small visual headroom while keeping scale centered on 0.
	maxAbs *= 1.10;
	yMin = -maxAbs;
	yMax = maxAbs;
}

void drawAxes() {
	float left = 120;
	float right = width - 60;
	float top = 60;
	float bottom = height - 130;

	noStroke();
	fill(PLOT_BG);
	rect(left, top, right - left, bottom - top);
	stroke(BORDER_COLOR);
	strokeWeight(1.2);
	noFill();
	rect(left, top, right - left, bottom - top);

	// Horizontal grid + y labels
	fill(TEXT_COLOR);
	textAlign(RIGHT, CENTER);
	float yStep = niceStep((yMax - yMin) / 4.0);
	for (float v = floor(yMin / yStep) * yStep; v <= yMax + 0.0001; v += yStep) {
		float y = map(v, yMax, yMin, top, bottom);
		stroke(abs(v) < 0.0001 ? GRID_ZERO_COLOR : GRID_COLOR);
		strokeWeight(abs(v) < 0.0001 ? 1.4 : 1.0);
		line(left, y, right, y);
		noStroke();
		text(nf(v, 1, 3), left - 10, y);
	}

	// Vertical grid for the 10-second sliding window
	textAlign(CENTER, TOP);
	for (int i = 0; i <= 5; i++) {
		float x = map(i * 2, 0, 10, left, right);
		stroke(i == 5 ? GRID_ZERO_COLOR : GRID_COLOR);
		strokeWeight(i == 5 ? 1.4 : 1.0);
		line(x, top, x, bottom);
		noStroke();
		text(-(10 - i * 2) + " s", x, bottom + 8);
	}

	fill(TEXT_COLOR);
	textAlign(CENTER, BOTTOM);
	pushMatrix();
	translate(42, (top + bottom) * 0.5);
	rotate(-HALF_PI);
	text("Acceleration (g)", 0, 0);
	popMatrix();
	textAlign(CENTER, TOP);
	text("Time (10 s sliding window)", (left + right) * 0.5, height - 45);
}

void drawCurves() {
	if (samples.size() < 2) {
		return;
	}

	float left = 120;
	float right = width - 60;
	float top = 60;
	float bottom = height - 130;

	float newestT = samples.get(samples.size() - 1).t;
	float startT = newestT - WINDOW_SECONDS;

	drawSingleCurve(left, right, top, bottom, startT, newestT, color(220, 35, 35), 'x');
	drawSingleCurve(left, right, top, bottom, startT, newestT, color(30, 130, 255), 'y');
	drawSingleCurve(left, right, top, bottom, startT, newestT, color(10, 170, 80), 'z');
}

void drawSingleCurve(float left, float right, float top, float bottom, float startT, float endT, int c, char axis) {
	stroke(c);
	strokeWeight(2.0);
	noFill();
	beginShape();
	for (Sample s : samples) {
		float x = map(s.t, startT, endT, left, right);
		float v = axis == 'x' ? s.ax : (axis == 'y' ? s.ay : s.az);
		float y = map(constrain(v, yMin, yMax), yMax, yMin, top, bottom);
		vertex(x, y);
	}
	endShape();
}

void drawLegendAndStatus() {
	fill(TEXT_COLOR);
	textAlign(LEFT, TOP);

	String status = (client != null && client.active()) ? "Connected" : "Disconnected";
	text("TCP " + status + "  |  " + ESP32_IP + ":" + ESP32_PORT, 120, 18);

	strokeWeight(3);
	stroke(TRACE_X);
	line(width - 360, 24, width - 320, 24);
	stroke(TRACE_Y);
	line(width - 250, 24, width - 210, 24);
	stroke(TRACE_Z);
	line(width - 140, 24, width - 100, 24);

	fill(TEXT_COLOR);
	noStroke();
	text("AX", width - 312, 14);
	text("AY", width - 202, 14);
	text("AZ", width - 92, 14);

	if (!samples.isEmpty()) {
		Sample last = samples.get(samples.size() - 1);
		text(
			"Last sample  ax=" + nf(last.ax, 1, 3) +
			"  ay=" + nf(last.ay, 1, 3) +
			"  az=" + nf(last.az, 1, 3) +
			"  |  sensor_ts=" + last.sensorTsUs + " us" +
			"  |  y-range=[" + nf(yMin, 1, 2) + ", " + nf(yMax, 1, 2) + "]",
			120,
			height - 60
		);
	}

	String sendingText = sendingFrequencyHz >= 0 ? nf(sendingFrequencyHz, 1, 0) + " Hz" : "N/A";
	float deltaHz = sendingFrequencyHz >= 0 ? sendingFrequencyHz - receivingFrequencyHz : 0;
	float lossPct = (sendingFrequencyHz > 0)
		? max(0, (sendingFrequencyHz - receivingFrequencyHz) / sendingFrequencyHz * 100.0)
		: 0;

	String debugText = "Sending=" + sendingText +
		"   Receiving=" + nf(receivingFrequencyHz, 1, 1) + " Hz" +
		(sendingFrequencyHz >= 0
			? "   Delta=" + nf(deltaHz, 1, 1) + " Hz   Estimated loss=" + nf(lossPct, 1, 1) + "%"
			: "   Waiting for SENDING_FREQUENCY packet...");

	text(debugText, 120, height - 30);
}

float niceStep(float rawStep) {
	float exponent = floor(log(rawStep) / log(10));
	float fraction = rawStep / pow(10, exponent);
	float niceFraction;

	if (fraction <= 1.0) {
		niceFraction = 1.0;
	} else if (fraction <= 2.0) {
		niceFraction = 2.0;
	} else if (fraction <= 5.0) {
		niceFraction = 5.0;
	} else {
		niceFraction = 10.0;
	}

	return niceFraction * pow(10, exponent);
}
