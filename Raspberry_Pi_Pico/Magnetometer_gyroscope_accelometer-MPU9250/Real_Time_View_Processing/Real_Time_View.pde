import processing.serial.*;

Serial myPort;  // Serial port object
float pitch = 0, roll = 0, yaw  = 0, temp = 0, humidity = 0; // Initialize pitch, roll, yaw

void setup() {
  size(1100, 900, P3D);  // Create a 3D window
  myPort = new Serial(this, "COM3", 115200);  // Replace "COM3" with your microcontroller's port
  myPort.bufferUntil('\n');  // Wait for a newline character in the serial data
}

void drawWheel(float x, float y, float z) {
  pushMatrix();
  translate(x, y, z);
  rotateZ(HALF_PI);  // Rotate wheel to lie flat on the ground
  cylinder(20, 30);  // Shorter height for realistic wheel thickness
  popMatrix();
}

// Improved Cylinder Function
void cylinder(float radius, float height) {
  int sides = 24;
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float angle = TWO_PI / sides * i;
    float x = cos(angle) * radius;
    float z = sin(angle) * radius;
    vertex(x, -height / 2, z);
    vertex(x, height / 2, z);
  }
  endShape();
}

void draw() {
  background(0, 50, 120); // Red-Green-Blue combination
  lights();  // Add lighting
  println("Pitch: " + pitch + ", Roll: " + roll + ", Yaw: " + yaw);


  // Draw the 3D car model
  pushMatrix();  // Save current transformation matrix
  translate(width / 2, height / 2, -200);  // Center the object
  rotateY(radians(-yaw));  // Rotate the car based on yaw
  rotateZ(radians(-pitch+15));  // Rotate the car based on pitch
  rotateX(radians(roll-15));  // Rotate the car based on roll

  // Car Body
  fill(0, 150, 100);  // Color
  box(200, 50, 300);  // Main car body

  // Car Roof
  fill(255, 255, 255);  // Color
  translate(0, -40, 0);  // Move up for the roof
  box(150, 40, 180);  // Roof

  // Wheels (Fixed Orientation)
  fill(50, 50, 50);
  drawWheel(-70, 80, 120);  // Front-left
  drawWheel(70, 80, 120);   // Front-right
  drawWheel(-70, 80, -120); // Rear-left
  drawWheel(70, 80, -120);  // Rear-right

  // Front Headlights
  translate(0, 30, 150); // Move to the front of the car
  fill(255, 255, 0);  // Yellow for headlights
  ellipse(-60, 0, 20, 20); // Left headlight
  ellipse(60, 0, 20, 20);  // Right headlight

  // Rear Taillights
  translate(0, -10, -140); // Move closer to the rear edge
  fill(255, 0, 0); // Red taillights
  sphere(10); // Spheres are affected by lighting better than ellipses
  ellipse(-60, 0, 20, 20); // Left taillight
  ellipse(60, 0, 20, 20);  // Right taillight
  popMatrix();  // Restore transformation matrix

  // Draw pitch and roll values (fixed at the bottom)
  fill(0);  // Black text
  textAlign(CENTER);
  textSize(20);
  text("Pitch: " + nf(pitch, 1, 1) + "°", width / 2, height - 80);  // Display pitch
  text("Roll: " + nf(roll, 1, 1) + "°", width / 2, height - 40);   // Display roll
  text("Yaw: " + nf(yaw, 1, 1) + "°", width / 2, height - 20);   // Display roll
  text("Temperature: " + nf(temp, 1, 1) + "°C", width / 2, height - 120);   // Display roll
  text("Humidity: " + nf(humidity, 1, 1) + "%", width / 2, height - 100);   // Display roll
}

void serialEvent(Serial myPort) {
    String dataString = myPort.readStringUntil('\n');  // Read data until newline
    if (dataString != null) {
        dataString = trim(dataString);  // Clean up the data
        println("Raw Data: [" + dataString + "]");  // Debugging raw data

        // Remove labels dynamically
        dataString = dataString.replaceAll("[a-zA-Z]+:", "").trim(); 
        println("Cleaned Data: [" + dataString + "]");  // Debugging cleaned data

        // Split the cleaned data into components
        String[] values = split(dataString, ',');  // Split by commas
        println("Values Length: " + values.length);

        if (values.length >= 3) { // Always update pitch, roll, yaw
            try {
                pitch = float(values[0].trim());
                roll = float(values[1].trim());
                yaw = float(values[2].trim());
            } catch (NumberFormatException e) {
                println("Error parsing pitch, roll, yaw: " + e.getMessage());
            }
        }

        if (values.length >= 5) { // Only update temp & humidity if present
            try {
                pitch = float(values[0].trim());
                roll = float(values[1].trim());
                yaw = float(values[2].trim());
                temp = float(values[3].trim());
                humidity = float(values[4].trim());
            } catch (NumberFormatException e) {
                println("Error parsing temp, humidity: " + e.getMessage());
            }
        }

        println("Parsed Data - Pitch: " + pitch + ", Roll: " + roll + ", Yaw: " + yaw + ", Temp: " + temp + ", Humidity: " + humidity);
    }
}
