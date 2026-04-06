import processing.net.*;

final String ESP32_IP = "192.168.4.1";
final int ESP32_PORT = 23;

final int BG_COLOR = color(6, 10, 8);
final int GRID_COLOR = color(25, 60, 35);
final int TEXT_COLOR = color(165, 225, 170);
final int CUBE_EDGE_COLOR = color(120, 255, 140);

Client client;
long lastConnectAttemptMs = 0;
final int RECONNECT_INTERVAL_MS = 1000;

float pitchDeg = 0;
float rollDeg = 0;
float yawDeg = 0;

float pendingPitchDeg = 0;
float pendingRollDeg = 0;
float pendingYawDeg = 0;
boolean hasPitch = false;
boolean hasRoll = false;
boolean hasYaw = false;

float sendingFrequencyHz = -1;
long lastPacketMs = 0;

void setup() {
  size(1280, 800, P3D);
  frameRate(60);
  smooth(2);
  textFont(createFont("Consolas", 18));
  surface.setTitle("KuDAQ - Orientation Viewer");
  connectIfNeeded();
}

void draw() {
  background(BG_COLOR);

  connectIfNeeded();
  readTcpStream();

  drawOrientedCube();
  drawHud();
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

  if (key.equals("PITCH")) {
    try {
      pendingPitchDeg = Float.parseFloat(valueText);
      hasPitch = true;
    }
    catch (Exception e) {
      return;
    }
  } else if (key.equals("ROLL")) {
    try {
      pendingRollDeg = Float.parseFloat(valueText);
      hasRoll = true;
    }
    catch (Exception e) {
      return;
    }
  } else if (key.equals("YAW")) {
    try {
      pendingYawDeg = Float.parseFloat(valueText);
      hasYaw = true;
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

  // Update orientation as soon as one complete P/R/Y vector is received.
  if (hasPitch && hasRoll && hasYaw) {
    pitchDeg = constrain(pendingPitchDeg, -180, 180);
    rollDeg = constrain(pendingRollDeg, -90, 90);

    // Normalize yaw to [0, 360).
    float y = pendingYawDeg % 360.0;
    if (y < 0) {
      y += 360.0;
    }
    yawDeg = y;

    hasPitch = false;
    hasRoll = false;
    hasYaw = false;
    lastPacketMs = millis();
  }
}



void drawOrientedCube() {
  pushMatrix();

  translate(width * 0.5, height * 0.5, 0);

  // Apply orientation: yaw, pitch, roll.
  rotateY(radians(yawDeg));
  rotateX(radians(pitchDeg));
  rotateZ(radians(rollDeg));

  float side = min(width, height) * 0.22;
  float half = side * 0.5;

  // Draw colored cube faces with light fill + dark edges for clarity.
  strokeWeight(1.5);
  stroke(CUBE_EDGE_COLOR);

  // Front face (Z+) - Bright red
  fill(220, 100, 80);
  beginShape();
  vertex(-half, -half, half);
  vertex(half, -half, half);
  vertex(half, half, half);
  vertex(-half, half, half);
  endShape(CLOSE);

  // Back face (Z-) - Dark red
  fill(140, 60, 50);
  beginShape();
  vertex(-half, -half, -half);
  vertex(-half, half, -half);
  vertex(half, half, -half);
  vertex(half, -half, -half);
  endShape(CLOSE);

  // Right face (X+) - Bright green
  fill(100, 220, 120);
  beginShape();
  vertex(half, -half, -half);
  vertex(half, half, -half);
  vertex(half, half, half);
  vertex(half, -half, half);
  endShape(CLOSE);

  // Left face (X-) - Dark green
  fill(60, 140, 80);
  beginShape();
  vertex(-half, -half, -half);
  vertex(-half, -half, half);
  vertex(-half, half, half);
  vertex(-half, half, -half);
  endShape(CLOSE);

  // Top face (Y+) - Bright blue
  fill(100, 140, 240);
  beginShape();
  vertex(-half, -half, -half);
  vertex(half, -half, -half);
  vertex(half, -half, half);
  vertex(-half, -half, half);
  endShape(CLOSE);

  // Bottom face (Y-) - Dark blue
  fill(70, 100, 180);
  beginShape();
  vertex(-half, half, -half);
  vertex(-half, half, half);
  vertex(half, half, half);
  vertex(half, half, -half);
  endShape(CLOSE);

  // Draw axes labels for reference.
  noFill();
  stroke(255, 200, 100);
  strokeWeight(2);
  // X axis (red marker)
  line(half * 0.6, 0, 0, half * 1.2, 0, 0);
  // Y axis (green marker)
  line(0, -half * 0.6, 0, 0, -half * 1.2, 0);
  // Z axis (blue marker)
  line(0, 0, half * 0.6, 0, 0, half * 1.2);

  popMatrix();
}

void drawHud() {
  fill(TEXT_COLOR);
  noStroke();
  textAlign(LEFT, TOP);

  String status = (client != null && client.active()) ? "Connected" : "Disconnected";
  text("TCP " + status + "  |  " + ESP32_IP + ":" + ESP32_PORT, 24, 18);

  text(
    "Pitch=" + nf(pitchDeg, 1, 3) + " deg   Roll=" + nf(rollDeg, 1, 3) + " deg   Yaw=" + nf(yawDeg, 1, 3) + " deg",
    24,
    48
  );

  if (sendingFrequencyHz >= 0) {
    text("Sending frequency: " + nf(sendingFrequencyHz, 1, 0) + " Hz", 24, 78);
  }

  if (lastPacketMs > 0) {
    float ageMs = millis() - lastPacketMs;
    text("Last packet age: " + nf(ageMs, 1, 0) + " ms", 24, 108);
  }

  textAlign(CENTER, TOP);
  text("Orientation ranges: Pitch [-180,180]   Roll [-90,90]   Yaw [0,360)", width * 0.5, height - 30);
}
