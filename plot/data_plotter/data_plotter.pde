import processing.net.*;
import java.io.File;
import java.io.PrintWriter;

final String ESP32_IP = "192.168.4.1";
final int ESP32_PORT = 23;

final int BG_COLOR = color(6, 10, 8);
final int GRID_COLOR = color(25, 60, 35);
final int TEXT_COLOR = color(165, 225, 170);
final int CUBE_EDGE_COLOR = color(120, 255, 140);
final int BUTTON_ON_COLOR = color(70, 170, 95);
final int BUTTON_OFF_COLOR = color(190, 85, 75);
final int BUTTON_DISABLED_COLOR = color(75, 90, 80);
final int BUTTON_TEXT_COLOR = color(240, 250, 240);

final float BUTTON_W = 150;
final float BUTTON_H = 44;
final float BUTTON_GAP = 16;
final float BUTTON_X = 24;
final float BUTTON_Y = 182;
final float RECORD_BUTTON_Y = 242;

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
boolean hasTimestamp = false;
boolean expectingTimestampValue = false;
long pendingTimestampMs = 0;

float sendingFrequencyHz = -1;
long lastPacketMs = 0;
boolean showCube = false;
String lastServerMessage = "-";
long lastServerMessageMs = 0;
boolean isRecording = false;
PrintWriter recordingWriter = null;
String recordingFileName = "-";

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

  if (showCube) {
    drawOrientedCube();
  }
  drawHud();
  drawButtons();
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
    sendStreamStateCommand();
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
  if (line.startsWith("OK ") || line.startsWith("ERR ") || line.equals("PONG")) {
    rememberServerMessage(line);
    return;
  }

  if (expectingTimestampValue) {
    try {
      pendingTimestampMs = Long.parseLong(line);
      hasTimestamp = true;
      expectingTimestampValue = false;
      commitOrientationSample();
    }
    catch (Exception e) {
      expectingTimestampValue = false;
    }
    return;
  }

  int sep = line.indexOf(':');
  if (sep <= 0) {
    rememberServerMessage(line);
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
  } else if (key.equals("TIMESTAMP")) {
    if (valueText.length() == 0) {
      expectingTimestampValue = true;
      return;
    }

    try {
      pendingTimestampMs = Long.parseLong(valueText);
      hasTimestamp = true;
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
  } else if (key.equals("CF") || key.equals("SR") || key.equals("STATE")) {
    rememberServerMessage(line);
    return;
  } else {
    rememberServerMessage(line);
    return;
  }

  commitOrientationSample();
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

  text("ESP message: " + lastServerMessage, 24, 138);
  text("Recording: " + (isRecording ? recordingFileName : "inactive"), 24, 168);

  textAlign(CENTER, TOP);
  text("Orientation ranges: Pitch [-180,180]   Roll [-90,90]   Yaw [0,360)", width * 0.5, height - 30);
}

void drawButtons() {
  drawButton(BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, "stream on", showCube, BUTTON_ON_COLOR);
  drawButton(BUTTON_X + BUTTON_W + BUTTON_GAP, BUTTON_Y, BUTTON_W, BUTTON_H, "stream off", !showCube, BUTTON_OFF_COLOR);
  drawButton(BUTTON_X, RECORD_BUTTON_Y, BUTTON_W, BUTTON_H, "start recording", showCube && !isRecording, BUTTON_ON_COLOR);
  drawButton(BUTTON_X + BUTTON_W + BUTTON_GAP, RECORD_BUTTON_Y, BUTTON_W, BUTTON_H, "stop recording", showCube && isRecording, BUTTON_OFF_COLOR);
}

void drawButton(float x, float y, float w, float h, String label, boolean isActive, int activeColor) {
  stroke(30, 45, 35);
  strokeWeight(1.5);
  fill(isActive ? activeColor : BUTTON_DISABLED_COLOR);
  rect(x, y, w, h, 8);

  fill(BUTTON_TEXT_COLOR);
  textAlign(CENTER, CENTER);
  text(label, x + w * 0.5, y + h * 0.5 - 1);
}

boolean isInside(float mx, float my, float x, float y, float w, float h) {
  return mx >= x && mx <= x + w && my >= y && my <= y + h;
}

void mousePressed() {
  if (isInside(mouseX, mouseY, BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H)) {
    showCube = true;
    sendStreamStateCommand();
    return;
  }

  float offX = BUTTON_X + BUTTON_W + BUTTON_GAP;
  if (isInside(mouseX, mouseY, offX, BUTTON_Y, BUTTON_W, BUTTON_H)) {
    stopRecording();
    showCube = false;
    sendStreamStateCommand();
    return;
  }

  if (!showCube) {
    return;
  }

  if (isInside(mouseX, mouseY, BUTTON_X, RECORD_BUTTON_Y, BUTTON_W, BUTTON_H)) {
    startRecording();
    return;
  }

  if (isInside(mouseX, mouseY, offX, RECORD_BUTTON_Y, BUTTON_W, BUTTON_H)) {
    stopRecording();
  }
}

void sendStreamStateCommand() {
  String command = showCube ? "stream orient on" : "stream orient off";
  sendTcpCommand(command);
}

void commitOrientationSample() {
  if (!(hasPitch && hasRoll && hasYaw && hasTimestamp)) {
    return;
  }

  pitchDeg = constrain(pendingPitchDeg, -180, 180);
  rollDeg = constrain(pendingRollDeg, -90, 90);

  float y = pendingYawDeg % 360.0;
  if (y < 0) {
    y += 360.0;
  }
  yawDeg = y;

  if (isRecording && recordingWriter != null) {
    recordingWriter.println(nf(pendingPitchDeg, 0, 3) + "," + nf(pendingRollDeg, 0, 3) + "," + nf(pendingYawDeg, 0, 3) + "," + pendingTimestampMs);
    recordingWriter.flush();
  }

  hasPitch = false;
  hasRoll = false;
  hasYaw = false;
  hasTimestamp = false;
  lastPacketMs = millis();
}

void startRecording() {
  if (!showCube || isRecording) {
    return;
  }

  File recordingFolder = new File(sketchPath("")).getParentFile();
  if (recordingFolder == null) {
    println("Cannot resolve plot folder for recording.");
    return;
  }

  String fileBaseName =
    "kuDAQ_recording_" +
    nf(year(), 4) + "-" + nf(month(), 2) + "-" + nf(day(), 2) + "_" +
    nf(hour(), 2) + "-" + nf(minute(), 2) + "-" + nf(second(), 2) +
    ".log";

  File outputFile = new File(recordingFolder, fileBaseName);

  try {
    recordingWriter = createWriter(outputFile.getAbsolutePath());
    recordingFileName = fileBaseName;
    isRecording = true;
    println("Recording started: " + outputFile.getAbsolutePath());
  }
  catch (Exception e) {
    recordingWriter = null;
    recordingFileName = "-";
    isRecording = false;
    println("Failed to start recording: " + e.getMessage());
  }
}

void stopRecording() {
  if (!isRecording) {
    return;
  }

  if (recordingWriter != null) {
    recordingWriter.flush();
    recordingWriter.close();
    recordingWriter = null;
  }

  isRecording = false;
  println("Recording stopped: " + recordingFileName);
}

void rememberServerMessage(String line) {
  lastServerMessage = line;
  lastServerMessageMs = millis();
  println("ESP32: " + line);
}

void sendTcpCommand(String command) {
  if (client == null || !client.active()) {
    println("Cannot send command, TCP client is not connected: " + command);
    return;
  }

  client.write(command + "\r\n");
  println("Sent command: " + command);
}
