import processing.serial.*;

Serial myPort;
String portName = "COM6";

float pitch = 0, roll = 0, yaw = 0;
float temperature = 0, pressure = 0, altitude = 0;
float ax = 0, ay = 0, az = 0;
float gpsLat = 0, gpsLng = 0, gpsAlt = 0, gpsSpeed = 0;
int   gpsSats = 0;

// Sensor status flags
float msaOK = 0, bmpOK = 0, bmmOK = 0, sdOK = 0;

// Offsets for zeroing
float offsetPitch = 0, offsetRoll = 0, offsetYaw = 0;

// Time values
int h = 0, m = 0, s = 0;

// Store GPS trail (緯度, 經度列表)
ArrayList<PVector> gpsTrail;

// Threshold for filtering GPS outliers (degrees)
final float MAX_GPS_JUMP = 0.0015; // approximately 150 m

void setup() {
  size(800, 600, P3D);
  println("Available serial ports:");
  println(Serial.list());

  myPort = new Serial(this, portName, 57600);
  myPort.bufferUntil('\n');

  gpsTrail = new ArrayList<PVector>();
}

void draw() {
  background(200);
  lights();

  // Compute tared/display angles
  float displayPitch = pitch - offsetPitch;
  float displayRoll  = roll  - offsetRoll;
  float displayYaw   = yaw   - offsetYaw;

  // Draw rocket in 3D, oriented by IMU data
  pushMatrix();
  translate(width/2, height/2, 0);
  rotateX(radians(-displayPitch));
  rotateZ(radians(displayRoll));
  rotateY(radians(displayYaw));
  drawRocket();
  popMatrix();

  // 1) Make a thread‐safe copy of gpsTrail
  ArrayList<PVector> trailCopy;
  synchronized(gpsTrail) {
    trailCopy = new ArrayList<PVector>(gpsTrail);
  }

  // 2) Draw GPS trail from that copy
  drawGpsTrail(trailCopy);

  // Text overlay: sensor data
  fill(0);
  textSize(20);
  text("Pitch:    " + nf(displayPitch, 1, 2) + "°", 20,  30);
  text("Roll:     " + nf(displayRoll,  1, 2) + "°", 20,  60);
  text("Yaw:      " + nf(displayYaw,   1, 2) + "°", 20,  90);
  text("Temp:     " + nf(temperature,   1, 2) + "°C", 20, 120);
  text("Pressure: " + nf(pressure,      1, 2) + " hPa", 20, 150);
  text("Altitude: " + nf(altitude,      1, 2) + " m",  20, 180);

  // GPS data text
  text("GPS Lat:   " + nf(gpsLat,   1, 6), 20, 220);
  text("GPS Lng:   " + nf(gpsLng,   1, 6), 20, 250);
  text("GPS Alt:   " + nf(gpsAlt,   1, 2) + " m", 20, 280);
  text("GPS Sats:  " + gpsSats,           20, 310);
  text("GPS Speed: " + nf(gpsSpeed, 1, 2) + " m/s", 20, 340);
  text("Time:      " 
       + nf(h, 2) + ":" 
       + nf(m, 2) + ":" 
       + nf(s, 2), 
       20, 380);

  // Sensor status display (top‐right corner)
  int sx = width - 200;
  int sy =  50;
  textSize(18);
  text("MSA301:  " + (msaOK == 1 ? "OK" : "✕"), sx, sy);
  text("BMP388:  " + (bmpOK == 1 ? "OK" : "✕"), sx, sy + 30);
  text("BMM150:  " + (bmmOK == 1 ? "OK" : "✕"), sx, sy + 60);
  text("SD Card: " + (sdOK  == 1 ? "OK" : "✕"), sx, sy + 90);
}

// 畫 GPS 軌跡（在右下角 200×200 區域內）
void drawGpsTrail(ArrayList<PVector> trail) {
  if (trail.size() < 2) return;

  // Recompute min/max lat & lng on “trail” each frame
  float tmpMinLat = Float.POSITIVE_INFINITY;
  float tmpMaxLat = Float.NEGATIVE_INFINITY;
  float tmpMinLng = Float.POSITIVE_INFINITY;
  float tmpMaxLng = Float.NEGATIVE_INFINITY;
  for (PVector v : trail) {
    tmpMinLat = min(tmpMinLat, v.x);
    tmpMaxLat = max(tmpMaxLat, v.x);
    tmpMinLng = min(tmpMinLng, v.y);
    tmpMaxLng = max(tmpMaxLng, v.y);
  }

  int mapX = width - 220;
  int mapY = height - 220;
  int mapW = 200;
  int mapH = 200;

  noFill();
  stroke(0);
  rect(mapX, mapY, mapW, mapH);

  stroke(255, 0, 0);
  noFill();
  beginShape();
  for (PVector v : trail) {
    float mappedX, mappedY;

    // If all longitudes are identical (zero‐range), place at center horizontally
    if (abs(tmpMaxLng - tmpMinLng) < 1e-6) {
      mappedX = mapX + mapW / 2.0;
    } else {
      mappedX = map(v.y, tmpMinLng, tmpMaxLng, mapX, mapX + mapW);
    }

    // If all latitudes are identical (zero‐range), place at center vertically
    if (abs(tmpMaxLat - tmpMinLat) < 1e-6) {
      mappedY = mapY + mapH / 2.0;
    } else {
      // Flip Y so that higher latitude is toward mapY (top)
      mappedY = map(v.x, tmpMinLat, tmpMaxLat, mapY + mapH, mapY);
    }

    vertex(mappedX, mappedY);
  }
  endShape();
}

void serialEvent(Serial p) {
  String inLine = p.readStringUntil('\n');
  if (inLine == null) return;
  inLine = trim(inLine);
  String[] tokens = split(inLine, ',');

  // 資料應包含 16 個逗號分隔欄位：tokens[0] ~ tokens[15]
  if (tokens.length == 16) {
    try {
      temperature = float(tokens[0]);
      yaw         = float(tokens[1]);
      pressure    = float(tokens[2]);
      altitude    = float(tokens[3]);
      ax          = float(tokens[4]);
      ay          = float(tokens[5]);
      az          = float(tokens[6]);
      sdOK        = int(float(tokens[7]));
      gpsLat      = float(tokens[8]);
      gpsLng      = float(tokens[9]);
      gpsAlt      = float(tokens[10]);
      gpsSpeed    = float(tokens[11]);
      gpsSats     = int(tokens[12]);

      h = int(tokens[13]);
      m = int(tokens[14]);
      s = int(tokens[15]);

      bmpOK = (pressure != 0.00) ? 1 : 0;
      bmmOK = (yaw      != 0.00) ? 1 : 0;
      msaOK = (roll     != 0.00) ? 1 : 0;

      pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
      roll  = atan2( ay,  az)               * 180.0 / PI;

      if (gpsSats > 0) {
        PVector newPos = new PVector(gpsLat, gpsLng);
        synchronized(gpsTrail) {
          if (gpsTrail.isEmpty()) {
            gpsTrail.add(newPos);
          } else {
            PVector last = gpsTrail.get(gpsTrail.size() - 1);
            float dLat = abs(gpsLat - last.x);
            float dLng = abs(gpsLng - last.y);
            if (dLat < MAX_GPS_JUMP && dLng < MAX_GPS_JUMP) {
              gpsTrail.add(newPos);
            }
          }
        }
      }

    } catch (Exception e) {
      println("Parse error: " + e.getMessage());
    }
  } else {
    println("Unexpected token count: " + tokens.length + " → " + inLine);
  }
}

void keyPressed() {
  if (key == 'z' || key == 'Z') {
    offsetPitch = pitch;
    offsetRoll  = roll;
    offsetYaw   = yaw;
    println("Orientation zeroed at → Pitch: "
      + offsetPitch + "°, Roll: " + offsetRoll
      + "°, Yaw: " + offsetYaw + "°");
  }
}

// --------------------------------------
// Helper: drawRocket()
// --------------------------------------
void drawRocket() {
  fill(200, 50, 50);
  noStroke();

  // Body: a stretched cylinder
  pushMatrix();
  scale(1, 2, 1);
  cylinder(20, 100);
  popMatrix();

  // Nose cone
  pushMatrix();
  translate(0, -60, 0);
  cone(20, 40);
  popMatrix();

  // Three fins, spaced 120° apart around the body
  for (int i = 0; i < 3; i++) {
    pushMatrix();
    rotateY(TWO_PI * i / 3);
    translate(15, 40, 0);
    rotateZ(HALF_PI);
    beginShape();
    vertex(0, 0);
    vertex(10, -20);
    vertex(-10, -20);
    endShape(CLOSE);
    popMatrix();
  }
}

// --------------------------------------
// Helper: cylinder(r, h)
//   Draws a vertical cylinder centered at (0,0,0)
// --------------------------------------
void cylinder(float r, float h) {
  int sides = 24;
  float angleStep = TWO_PI / sides;

  // Side surface
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, -h/2, z);
    vertex(x,  h/2, z);
  }
  endShape();

  // Draw top and bottom caps as triangle fans
  for (int sign = -1; sign <= 1; sign += 2) {
    beginShape(TRIANGLE_FAN);
    vertex(0, sign * h/2, 0);
    for (int i = 0; i <= sides; i++) {
      float angle = i * angleStep;
      float x = cos(angle) * r;
      float z = sin(angle) * r;
      vertex(x, sign * h/2, z);
    }
    endShape();
  }
}

// --------------------------------------
// Helper: cone(r, h)
//   Draws a cone whose base is centered at (0,0,0) and extends
//   upward along –Y by height h.
// --------------------------------------
void cone(float r, float h) {
  int sides = 24;
  float angleStep = TWO_PI / sides;

  beginShape(TRIANGLE_FAN);
  vertex(0, -h, 0); 
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, 0, z);
  }
  endShape();
}
