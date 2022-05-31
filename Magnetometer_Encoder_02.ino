#include <Chrono.h>
#include <Arduino_JSON.h>


Chrono chrono_SerialSend;

Chrono chrono_SensorRead_micros(Chrono::MICROS);

Chrono my_Chrono_1second;

int samplerate = 4; // in milliseconds
int SerialSendInterval = 20; // milliseconds

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define PCA_PC  (2)
#define PCA_OBS (50)
#define PCA_VAR (3)

BLA::Matrix<PCA_OBS, PCA_VAR> calData;
BLA::Matrix<1, PCA_VAR> centroid; // in 3d (based on cal data)
BLA::Matrix<PCA_OBS, PCA_PC> T; // T: scores or transformed coordinates.    dim: Datapoints x PC
BLA::Matrix<PCA_VAR, PCA_PC> P; // P: loadings or coefficients.             dim: Variable count x PC

// Circle fit in the PC space. Assigned in step 40.
float circ_x;
float circ_y;
float circ_r;

// Variable updated every cycle:
//BLA::Matrix<1, PCA_VAR> instance; // Single instance of magnetic measurement.
float pc1_score_cur = 0;
float pc2_score_cur = 0;
float angle_cur = 0;
float angle_prev = 0;
float angle_speed = 0;

// Ellipse scale factors:
float xscale;
float yscale;

// Gradient descent variables
float f = 0;
float smallstep = 0.000001;
float f_a = 0;
float f_b = 0;
float df_da = 0;
float df_db = 0;
// </GD variables



#include <Arduino_LSM9DS1.h>
float x, y, z;


int progState = 0;

float pi_half = 1.570796;

int loopCounter = 0;


const int pwmPin = 13;
const int dirPin = 12;

float angle = 0;

int externalMotorSignal = 0;



// Further work:
//Maksimal målehastighet med arduino: 110 rad/s (vises som 11 pga faktor på 100 som skulle vært 1000)

//110 rad/s  --> 1050 RPM.

//Hvis samplingsraten kan bli raskere enn 24 ms kan vi måle høyere hastigheter.

//Korriger for ustabil hastighetsmåling ved å flytte senter av sirkelen litt.
//Enten circle fit eller iterativt flytte sirkelens senter mot steder der det er lav hastighet (De er sannsynligvis for nære)




void setup() {
  Serial.begin(9600);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);



  while (!Serial) {
  }
  //IMU.setMagnetODR(....);
  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  IMU.setMagnetODR(8);  // Magnet ODR is set to 400 hz


}

void loop() {

  if (progState == 0) {
    // Collect data
    CollectCalData();
    progState = 10;
  }
  if (progState == 10) {
    // print data
    //Serial << calData;
    progState = 20;
  }
  if (progState == 20) {
    // Subtract the centroid
    Centroid(calData, centroid );               // Get the centroid
    // Subtract the centroid
    for ( int i = 0; i < PCA_VAR; i++) {
      calData.Submatrix<PCA_OBS, 1>(0, i) -=  centroid(0, i);
    }
    progState = 30;

    //Serial << calData;
  }
  if (progState == 30) {
    // Compute Nipals
    int iterations = Nipals(calData, T, P);                           // Run NIPALS algorithm
    //Serial << T;
    progState = 40;
  }

  if (progState == 40) {
    // Compute circle center in PC space
    float x[PCA_OBS];
    float y[PCA_OBS];
    for ( int i = 0; i < PCA_OBS; i++) {
      x[i] = T(i, 0);
      y[i] = T(i, 1);
    }
    FindCircleCenter(x, y, PCA_OBS, circ_x, circ_y, circ_r);
    //    Serial.println("");
    //    Serial.println("Circle that fits the data best: ");
    //
    //    Serial.print("X coord: ");
    //    Serial.println(circ_x);
    //    Serial.print("Y coord: ");
    //    Serial.println(circ_y);
    //    Serial.print("Radius: ");
    //    Serial.println(circ_r);
    //
    //
    //    T.Submatrix<PCA_OBS, 1>(0, 0) -=  circ_x;
    //    T.Submatrix<PCA_OBS, 1>(0, 1) -=  circ_y;
    //    Serial << T;
    progState = 50;
  }
  if (progState == 50) {
    // Find the scale factors for an ellipse fit.

    // transform to centered "unit ellipse".
    float cent_x[PCA_OBS];
    float cent_y[PCA_OBS];
    for (int i = 0; i < PCA_OBS; i++) {
      cent_x[i] = (T(i, 0) - circ_x) / circ_r;
      cent_y[i] = (T(i, 1) - circ_y) / circ_r;
    }

    // Find ellipse scale factors:
    FindEllipsefactors(cent_x, cent_y, PCA_OBS, xscale, yscale);


    progState = 100;
  }
  //  if (progState == 60) {
  //    // Try to find center again (did not seem to help)
  //    float comp_x[PCA_OBS];
  //    float comp_y[PCA_OBS];
  //    for (int i = 0; i < PCA_OBS; i++) {
  //      comp_x[i] = (T(i, 0) - circ_x) / xscale ;
  //      comp_y[i] = (T(i, 1) - circ_y) / yscale;
  //    }
  //
  //    float temp_x = 0;
  //    float temp_y = 0;
  //    float temp_r = 0;
  //    FindCircleCenter(comp_x, comp_y, PCA_OBS, temp_x, temp_y, temp_r); // improve the center positions
  //
  //    Serial.print("comp_X, ");
  //        Serial.print(temp_x);
  //        Serial.print(", comp_y, ");
  //        Serial.print(temp_y);
  //        Serial.print(", ");
  //
  //    progState = 100;
  //  }

  if (progState == 100) {
    // Normal loop

    // Collect data
    if (chrono_SensorRead_micros.hasPassed(samplerate * 1000)) {
      if (IMU.magnetAvailable()) { // The interval ssems to be 24 ms for the magnetometer...
        int dt = chrono_SensorRead_micros.elapsed();

        //        Serial.print(((float)dt) / 10000);
        //        Serial.print(", " );
        chrono_SensorRead_micros.restart();


        //chrono_oldINterval.restart();

        IMU.readMagnet(x, y, z);

        GetPrincipalComponents(x - centroid(0, 0), y - centroid(0, 1), z - centroid(0, 2), pc1_score_cur, pc2_score_cur);
        angle_prev = angle_cur;
        angle_cur = atan2(pc1_score_cur / xscale, pc2_score_cur / yscale) + PI; // Angle becomes 0 to 2*pi; // xscale and yscale is used to compensate for the oval shape.

        //        Serial.print("PC1, ");
        //        Serial.print(pc1_score_cur / xscale);
        //        Serial.print(", PC2, ");
        //        Serial.print(pc2_score_cur / yscale);
        //        Serial.print(", ");



        float angle_dif = angle_cur - angle_prev;
        if (angle_dif > PI) angle_dif = -(2 * PI - angle_dif);
        if (angle_dif < -PI) angle_dif = -(-2 * PI - angle_dif) ;

        angle_speed = 1000 * 100.0 * angle_dif / ((float)dt);
        //        Serial.print(angle_cur);
        //        Serial.print(" , ");
        //        Serial.println(angle_speed);

        loopCounter += 1;
      }
    }


    // Send state over serial port:
    if (chrono_SerialSend.hasPassed(SerialSendInterval)) {
      chrono_SerialSend.restart();

      JSONVar myObject;
      myObject["speed"] = round2(angle_speed);
      myObject["angle"] = round2(angle_cur);
      //Serial.println(myObject.keys());
      Serial.println(myObject);
    }

    String msg = "";
    char character;
    while (Serial.available()) {
      character = Serial.read();
      if (character == '\n') {
        HandleJson(msg); // Set motor speed
        msg = "";
      }
      else {
        msg.concat(character);
      }

    }


    //const char input[] = "{\"result\":true,\"count\":42,\"foo\":\"bar\"}";
    //    JSONVar myObject = JSON.parse(input);
    //    if (myObject.hasOwnProperty("mv")) {
    //      externalMotorSignal = (int) myObject["mv"]
    //
    //                            RunMotor(externalMotorSignal);
    //    }

    // Run the motor
    //    angle += 0.01;
    //    if (angle > 2 * PI) {
    //      angle = angle - 2 * PI;
    //    }
    //
    //    float mysignal = sin(angle) * 200;
    //
    //
    //    RunMotor((int)mysignal);





    progState = 100;
  }

  //  if ( my_Chrono_1second.hasPassed(1000)) {
  //    my_Chrono_1second.restart();
  //    Serial.print("Number of readings last second: ");
  //    Serial.println(loopCounter);
  //    loopCounter = 0;
  //  }


}



















void HandleJson(String msg) {
  JSONVar myObject = JSON.parse(msg);
  if (myObject.hasOwnProperty("mv")) {
    externalMotorSignal = (int) myObject["mv"];

    RunMotor(externalMotorSignal);
    //Serial.println(externalMotorSignal);
  }
}

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}


void RunMotor(int mspeed ) {
  if (mspeed > 0) {
    digitalWrite(dirPin, HIGH);
  }
  else {
    digitalWrite(dirPin, LOW);
  }

  analogWrite(pwmPin, abs(mspeed));
}


float EllipseCost(float points_x[], float points_y[], int num_p, float a, float b) {
  // Assumes a "unit ellipse" (roughly scaled to average radius of 1).
  float sum = 0;
  float avg_scale = 0;
  for (int i = 0; i < num_p; i++)
  {
    // The method of computing average is not correct, but works ok. Should be changed in the future.
    avg_scale = 1.0 / (i + 1.0);
    sum += avg_scale * ((a * points_x[i] * a * points_x[i] + b * points_y[i] * b * points_y[i] - 1) * (a * points_x[i] * a * points_x[i] + b * points_y[i] * b * points_y[i] - 1));
  }
  return sum; // the sum is actually average cost for each point.
}


void GD_stepForward(float points_x[], float points_y[], int num_p, float learningrate, float& a, float& b) {
  f = EllipseCost(points_x, points_y, num_p, a, b);

  f_a = EllipseCost(points_x, points_y, num_p, a + smallstep, b);
  f_b = EllipseCost(points_x, points_y, num_p, a, b + smallstep);

  df_da = (f_a - f) / smallstep;
  df_db = (f_b - f) / smallstep;
  //Serial.println("cost: ");
  //Serial.println(f, 5);
  a -= df_da * learningrate;
  b -= df_db * learningrate;
}

void FindEllipsefactors(float data_x[], float data_y[], int num, float& xscale_out, float& yscale_out) {
  float a = 1;
  float b = 1;
  double learningRate = 0.02; // learning rate of 0.02 confirmed stable with oval shape (y = 2*x). (with average r = 1)
  for (int i = 0; i < 50; i++)
  {
    GD_stepForward(data_x, data_y, num, learningRate, a, b);
  }
  xscale_out = 1.0 / a;
  yscale_out = 1.0 / b;
}


float GetPrincipalComponents(float x, float y, float z, float& pc1, float& pc2) {
  //    float pc1_score_cur = (~P.Submatrix<PCA_VAR, 1>(0, 0) * ~instance)(0, 0); // first col in P transposed * a row in newdata transposed;
  //    float pc2_score_cur = (~P.Submatrix<PCA_VAR, 1>(0, 1) * ~instance)(0, 0); // second col in P transposed * a row in newdata transposed;
  pc1 = P(0, 0) * x + P(1, 0) * y + P(2, 0) * z;
  pc2 = P(0, 1) * x + P(1, 1) * y + P(2, 1) * z;
  pc1 -= circ_x;
  pc2 -= circ_y;
}

void Centroid(BLA::Matrix<PCA_OBS, PCA_VAR> data, BLA::Matrix<1, PCA_VAR> & centroid_ ) {
  centroid_.Fill(0);
  for (int i = 0; i < PCA_OBS; i++) {
    float weight = 1.0 / (i + 1);
    centroid_ += (data.Submatrix<1, PCA_VAR>(i, 0) - centroid_) * weight;
  }
}

void CollectCalData() {
  calData.Fill(0);
  int nr = 0;
  while (nr < PCA_OBS) {

    if (IMU.magnetAvailable()) {
      IMU.readMagnet(x, y, z);
      calData(nr, 0) = x;
      calData(nr, 1) = y;
      calData(nr, 2) = z;
      nr += 1;
      delay(40);
    }
  }
}
int Nipals(BLA::Matrix<PCA_OBS, PCA_VAR> Xh, BLA::Matrix<PCA_OBS, PCA_PC> & T, BLA::Matrix<PCA_VAR, PCA_PC> & P) {
  // The function assumes already centered data.
  int it_total = 0;
  const int it = 1000; // Maximum iterations per PC
  const float tol = 1e-4; // Tolerance

  //T.Fill(0);
  //P.Fill(0);
  BLA::Matrix<PCA_OBS, 1> thnew;

  int nr = 0;
  for (int h = 0; h < PCA_PC; h++) {  // Loop for each principal component
    BLA::Matrix<PCA_OBS, 1> th =  Xh.Column(0);
    BLA::Matrix<PCA_VAR, 1> ph;
    ph.Fill(0);
    bool ende = false;
    while (!ende) {   // loop until the change in p (loadings) gets small.
      nr += 1;
      ph = ~Xh * th / (~th * th)(0);
      ph = ph / Norm(ph);
      thnew = Xh * ph / (~ph * ph)(0);
      float prec = (~(thnew - th) * (thnew - th))(0);
      th = thnew;
      if (prec <= tol * tol) {
        ende = true;
      }
      else if (it <= nr) {
        ende = true;
        Serial.println("PCA: no convergence");
      }
    }
    Xh = Xh - th * ~ph;
    T.Submatrix<PCA_OBS, 1>(0, h) = th;
    P.Submatrix<PCA_VAR, 1>(0, h) = ph;

    it_total += nr;
    nr = 0;
  }
  return it_total;
}
void FindCircleCenter(float x[], float y[], int count, float& out_x, float& out_y, float& out_r) {

  float sum_xx = 0;
  float sum_xy = 0;
  float sum_x = 0;
  float sum_yy = 0;
  float sum_y = 0;

  float sum_x_xx_yy = 0;
  float sum_y_xx_yy = 0;
  float sum_xx_yy = 0;

  for ( int i = 0; i < count; i++) {
    sum_xx = sum_xx + x[i] * x[i];
    sum_xy = sum_xy + x[i] * y[i];
    sum_x = sum_x + x[i];
    sum_yy = sum_yy + y[i] * y[i];
    sum_y = sum_y + y[i];

    sum_x_xx_yy = sum_x_xx_yy + x[i] * (x[i] * x[i] + y[i] * y[i]);
    sum_y_xx_yy = sum_y_xx_yy + y[i] * (x[i] * x[i] + y[i] * y[i]);
    sum_xx_yy = sum_xx_yy + x[i] * x[i] + y[i] * y[i];
  }

  BLA::Matrix<3, 3> lhs = {sum_xx, sum_xy, sum_x, sum_xy, sum_yy, sum_y, sum_x, sum_y, count};
  BLA::Matrix<3, 1> rhs = {sum_x_xx_yy, sum_y_xx_yy, sum_xx_yy};
  Invert(lhs);
  BLA::Matrix<3, 1> result = lhs * rhs;
  float a = result(0); // coeff
  float b = result(1); // coeff
  float c = result(2); // coeff

  out_x = a / 2; // x coordinate of center
  out_y = b / 2; // y coordinate of center
  out_r = (sqrt(4 * c + a * a + b * b)) / 2; // Radius of circle
}
