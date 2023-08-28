#include <Wire.h>
#include "MAX30105.h"
#include "SparkFunHTU21D.h"
#include "heartRate.h"
#include <File.h>
#include <SDHCI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <GNSS.h>

static SpGnss Gnss;
Adafruit_MPU6050 mpu;
HTU21D myHumidity;
SpNavData NavData;
SDClass theSD;

unsigned int prevTime = 0, delayTime = 5000;
long timer = 0;
void WriteHeadings()
{
  File myFile = theSD.open("TEMP_SAT.csv", FILE_WRITE);
  delay(10);
  while (!Serial)
  {
  }
  if (myFile)
  {
    myFile.print("Time");
    myFile.print(",");
    myFile.print("Temperature");
    myFile.print(",");
    myFile.print("Humidity");
    myFile.print(",");
    myFile.print("Latitude");
    myFile.print(",");
    myFile.print("Longitude");
    myFile.print(",");
    myFile.print("Number of Satellites");
  }
  myFile.println();
  /* Close the file */
  myFile.close();
  while (!Serial)
  {
    /* wait for serial port to connect. Needed for native USB port only */
  }

  File myFile2 = theSD.open("IMU_ARRAY.csv", FILE_WRITE);// ...........................................Used to write headers of the second File
  delay(10);
  if (myFile2)
  {
    myFile2.print("Time");
    myFile2.print(",");
    myFile2.print("Acceleration (X)");
    myFile2.print(",");
    myFile2.print("Acceleration (Y)");
    myFile2.print(",");
    myFile2.print("Acceleration (Z)");
    myFile2.print(",");
    myFile2.print("Gyro (X)");
    myFile2.print(",");
    myFile2.print("Gyro (Y)");
    myFile2.print(",");
    myFile2.print("Gyro (Z)");
  }
  myFile2.println();
  myFile2.close();
}

void setupForMPU()
{
  Serial.begin(115200);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      //delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}


void setupForTemp() //Used to setup the temperature sensor
{
  Serial.begin(115200);
  int ret = 0;
  int subid;
  Serial.println("HTU21D Example!");
  myHumidity.begin();
  delay(1000);
  Serial.println("Initializing...");
}


void setupForGPS()
{
  /* Setup serial output for printing. */
  Serial.begin(115200);
  /* Initialize GNSS. */
  Gnss.begin();
  Gnss.start();
}

unsigned long prevTimeTEMPSAT = 0;
const long eventTimeTIMESAT = 1000; //in ms
int counter = 0;

void WriteTempSat() //This is used to prepare the SD card for reading and entering the data
{
  unsigned long currTimeT = millis();
  if (currTimeT - prevTimeTEMPSAT  >= eventTimeTIMESAT)
  {
    prevTimeTEMPSAT = currTimeT;
    //.......................................Used to get the Humidity values
    float humd = myHumidity.readHumidity();
    float temp = myHumidity.readTemperature();
    Gnss.getNavData(&NavData);
    //.......................................................Get GPS value
    File myFile = theSD.open("TEMP_SAT.csv", FILE_WRITE);
    if (myFile) {
      Serial.print("Writing to TEMP_SAT.csv...");
      myFile.print(millis());
      myFile.print(",");
      myFile.print(temp, 1);
      myFile.print(",");
      myFile.print(humd, 1);
      myFile.print(",");
      if (NavData.latitude == 0)
      {
        myFile.print("Could not get a lock on satellites. Please check weather....");
      }
      else
      {
        myFile.print(NavData.latitude, 6);
        myFile.print(",");
        myFile.print(NavData.longitude, 6);
        myFile.print(",");
        myFile.print(NavData.numSatellites);
      }
      myFile.println();
      /* Close the file */
      myFile.close();
      Serial.println("done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening TEMP_SAT.txt");
    }
    counter++;
  }
  if (counter == 1) {
    WriteArray();
  }
  else if (counter == 2) {
    performFFT();
  }
  else if (counter == 3) {
    writeFFT();
  }
  else if (counter == 10) {
    counter = 0;
  }
}

int saveFFT = 0;
//float ff[512] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511};
float ffr[256] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255};
float ffi[256] = {0};
int N = 256;
float x[128];
long unsigned axft[256];
int FFTdone = 0;

void performFFT() {
  if (FFTdone == 0) {
    int L = 256;
    const float dt = 0.02; // sample period (s)

    // FFT algorithm
    for (int j = 1; j <= 8; j++) {
      int fac = pow(2, j);
      int varj = N / fac;
      int Tj[varj];
      int Tjj[varj];
      for (int i = 0; i < varj; i++) {
        Tj[i] = i + 1;
        Tjj[i] = Tj[i] + varj;
      }

      int fac2 = pow(2, j - 1);
      int jm = N / fac2;
      float Wr[jm / 2];
      float Wi[jm / 2];
      for (int i = 0; i < jm / 2; i++) {
        Wr[i] = cos(2 * PI / N * pow(2, j - 1) * i); // real value
        Wi[i] = -sin(2 * PI / N * pow(2, j - 1) * i); // imaginary value
      }
      float FFr[N]; float FFi[N];
      for (int jn = 0; jn < fac2; jn++) {
        for (int i = 0; i < jm / 2; i++) {
          int k[jm / 2];
          int l[jm / 2];
          for (int m = 0; m < varj; m++) {
            Tj[m] = m + 1;
            Tjj[m] = Tj[m] + varj;
          }
          k[i] = Tj[i] + jn * jm - 1;
          l[i] = Tjj[i] + jn * jm  - 1;
          float Fkr; float Fki;
          Fkr = ffr[k[i]] + ffr[l[i]];
          Fki = ffi[k[i]] + ffi[l[i]];
          FFr[k[i]] = Fkr;
          FFi[k[i]] = Fki;
          float Flr;
          float a; float b; float c; float d;
          a = ffr[k[i]] - ffr[l[i]];
          b = ffi[k[i]] - ffi[l[i]];
          Flr = a * Wr[i] - b * Wi[i];
          float Fli;
          Fli = a * Wi[i] + b * Wr[i];
          FFr[l[i]] = Flr;
          FFi[l[i]] = Fli;

          ////          // Use To Debug
          //          Serial.print("i:");
          //          Serial.print("\t j:");
          //          Serial.print("\t jn:");
          //          //          Serial.print("\t Tj[i]:");
          //          //          Serial.print("\t Tjj[i]:");
          //          Serial.print("\t jm:");
          //          Serial.print("\t k[i]: ");
          //          Serial.print("\t l[i]: ");
          //          Serial.print("\t FFr[k[i]]:");
          //          Serial.print("\t FFi[k[i]]:");
          //          Serial.print("\t FFr[l[i]]:");
          //          Serial.println("\t FFi[l[i]]:");
          //          Serial.print(i);
          //          Serial.print("\t");
          //          Serial.print(j);
          //          Serial.print("\t");
          //          Serial.print(jn);
          //          Serial.print("\t");
          //          //          Serial.print(Tj[i]);
          //          //          Serial.print("\t");
          //          //          Serial.print(Tjj[i]);
          //          //          Serial.print("\t \t");
          //          Serial.print(jm);
          //          Serial.print("\t");
          //          Serial.print(k[i]);
          //          Serial.print("\t");
          //          Serial.print(l[i]);
          //          Serial.print("\t");
          //          Serial.print(FFr[k[i]]);
          //          Serial.print("\t \t");
          //          Serial.print(FFi[k[i]]);
          //          Serial.print("\t \t");
          //          Serial.print(FFr[l[i]]);
          //          Serial.print("\t \t");
          //          Serial.println(FFi[l[i]]);
        }
      }
      for (int i = 0; i < N; i++) {
        ffr[i] = FFr[i];
        ffi[i] = FFi[i];
      }
    }
    // Sequencing
    //int seq[512] = {1, 257, 129, 385, 65, 321, 193, 449, 33, 289, 161, 417, 97, 353, 225, 481, 17, 273, 145, 401, 81, 337, 209, 465, 49, 305, 177, 433, 113, 369, 241, 497, 9, 265, 137, 393, 73, 329, 201, 457, 41, 297, 169, 425, 105, 361, 233, 489, 25, 281, 153, 409, 89, 345, 217, 473, 57, 313, 185, 441, 121, 377, 249, 505, 5, 261, 133, 389, 69, 325, 197, 453, 37, 293, 165, 421, 101, 357, 229, 485, 21, 277, 149, 405, 85, 341, 213, 469, 53, 309, 181, 437, 117, 373, 245, 501, 13, 269, 141, 397, 77, 333, 205, 461, 45, 301, 173, 429, 109, 365, 237, 493, 29, 285, 157, 413, 93, 349, 221, 477, 61, 317, 189, 445, 125, 381, 253, 509, 3, 259, 131, 387, 67, 323, 195, 451, 35, 291, 163, 419, 99, 355, 227, 483, 19, 275, 147, 403, 83, 339, 211, 467, 51, 307, 179, 435, 115, 371, 243, 499, 11, 267, 139, 395, 75, 331, 203, 459, 43, 299, 171, 427, 107, 363, 235, 491, 27, 283, 155, 411, 91, 347, 219, 475, 59, 315, 187, 443, 123, 379, 251, 507, 7, 263, 135, 391, 71, 327, 199, 455, 39, 295, 167, 423, 103, 359, 231, 487, 23, 279, 151, 407, 87, 343, 215, 471, 55, 311, 183, 439, 119, 375, 247, 503, 15, 271, 143, 399, 79, 335, 207, 463, 47, 303, 175, 431, 111, 367, 239, 495, 31, 287, 159, 415, 95, 351, 223, 479, 63, 319, 191, 447, 127, 383, 255, 511, 2, 258, 130, 386, 66, 322, 194, 450, 34, 290, 162, 418, 98, 354, 226, 482, 18, 274, 146, 402, 82, 338, 210, 466, 50, 306, 178, 434, 114, 370, 242, 498, 10, 266, 138, 394, 74, 330, 202, 458, 42, 298, 170, 426, 106, 362, 234, 490, 26, 282, 154, 410, 90, 346, 218, 474, 58, 314, 186, 442, 122, 378, 250, 506, 6, 262, 134, 390, 70, 326, 198, 454, 38, 294, 166, 422, 102, 358, 230, 486, 22, 278, 150, 406, 86, 342, 214, 470, 54, 310, 182, 438, 118, 374, 246, 502, 14, 270, 142, 398, 78, 334, 206, 462, 46, 302, 174, 430, 110, 366, 238, 494, 30, 286, 158, 414, 94, 350, 222, 478, 62, 318, 190, 446, 126, 382, 254, 510, 4, 260, 132, 388, 68, 324, 196, 452, 36, 292, 164, 420, 100, 356, 228, 484, 20, 276, 148, 404, 84, 340, 212, 468, 52, 308, 180, 436, 116, 372, 244, 500, 12, 268, 140, 396, 76, 332, 204, 460, 44, 300, 172, 428, 108, 364, 236, 492, 28, 284, 156, 412, 92, 348, 220, 476, 60, 316, 188, 444, 124, 380, 252, 508, 8, 264, 136, 392, 72, 328, 200, 456, 40, 296, 168, 424, 104, 360, 232, 488, 24, 280, 152, 408, 88, 344, 216, 472, 56, 312, 184, 440, 120, 376, 248, 504, 16, 272, 144, 400, 80, 336, 208, 464, 48, 304, 176, 432, 112, 368, 240, 496, 32, 288, 160, 416, 96, 352, 224, 480, 64, 320, 192, 448, 128, 384, 256, 512};
    int seq[N] = {1, 129, 65, 193, 33, 161, 97, 225, 17, 145, 81, 209, 49, 177, 113, 241, 9, 137, 73, 201, 41, 169, 105, 233, 25, 153, 89, 217, 57, 185, 121, 249, 5, 133, 69, 197, 37, 165, 101, 229, 21, 149, 85, 213, 53, 181, 117, 245, 13, 141, 77, 205, 45, 173, 109, 237, 29, 157, 93, 221, 61, 189, 125, 253, 3, 131, 67, 195, 35, 163, 99, 227, 19, 147, 83, 211, 51, 179, 115, 243, 11, 139, 75, 203, 43, 171, 107, 235, 27, 155, 91, 219, 59, 187, 123, 251, 7, 135, 71, 199, 39, 167, 103, 231, 23, 151, 87, 215, 55, 183, 119, 247, 15, 143, 79, 207, 47, 175, 111, 239, 31, 159, 95, 223, 63, 191, 127, 255, 2, 130, 66, 194, 34, 162, 98, 226, 18, 146, 82, 210, 50, 178, 114, 242, 10, 138, 74, 202, 42, 170, 106, 234, 26, 154, 90, 218, 58, 186, 122, 250, 6, 134, 70, 198, 38, 166, 102, 230, 22, 150, 86, 214, 54, 182, 118, 246, 14, 142, 78, 206, 46, 174, 110, 238, 30, 158, 94, 222, 62, 190, 126, 254, 4, 132, 68, 196, 36, 164, 100, 228, 20, 148, 84, 212, 52, 180, 116, 244, 12, 140, 76, 204, 44, 172, 108, 236, 28, 156, 92, 220, 60, 188, 124, 252, 8, 136, 72, 200, 40, 168, 104, 232, 24, 152, 88, 216, 56, 184, 120, 248, 16, 144, 80, 208, 48, 176, 112, 240, 32, 160, 96, 224, 64, 192, 128, 256};
    // Reorder FFT result
    float Fr[N]; float Fi[N];
    for (int i = 0; i < N; i++)
    {
      Fr[i] = ffr[seq[i] - 1];
      Fi[i] = ffi[seq[i] - 1];
    }

    // Calculate amplitude spectrum
    for (int i = 0; i < N / 2; i++) {
      axft[i] = sqrt(sq(Fr[i]) + sq(Fi[i])) * 2 / L;

      //      Serial.print("axft[i]");
      //      Serial.print("\t");
      //      Serial.println(axft[i]);
      //      Serial.print(Fr[i]);
      //      Serial.print("\t");
      //      Serial.println(Fi[i]);
    }

    const float fc = 1 / (2 * dt);
    for (int i = 0; i < N / 2 ; i++) {
      x[i] = 2 * i * fc / N;
    }
    FFTdone = 1;
    Serial.println("FFT computed");
  }
}


void writeFFT() {
  if (saveFFT == 0) {
    //unsigned long currTimea = millis();
    //if (currTimea - prevTimeARR  >= eventTimeARR)
    //{
    /* Open the file. Note that only one file can be open at a time,
       so you have to close this one before opening another. */
    File myFile = theSD.open("FFT_ax.csv", FILE_WRITE);
    //delay(10);
    /* If the file opened okay, write to it */
    if (myFile) {
      Serial.print("Writing to FFT_ax.csv...");
      for (int i = 0; i < 128; i++) {
        myFile.print(x[i]);
        myFile.print(",");
        myFile.println(axft[i]);
      }
      /* Close the file */
      myFile.close();
      Serial.println("done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening FFT_ax.csv");
    }
    saveFFT = 1;
  }
}
/* When did the sensor start */
unsigned long prevTimeIMU = 0;
/* "independant" timed event */
const long eventTimeIMU = 19; //in ms 50 Hz
int countimu = 0;
unsigned long timeStamp;
unsigned long timea[512];
float axa[512];
float aya[512];
float aza[512];
float gxa[512];
float gya[512];
float gza[512];
int runOnce = 0;

void ReadIMU() //This is used to prepare the SD card for reading and enter the data
{
  sensors_event_t a, g, temp2;
  mpu.getEvent(&a, &g, &temp2);
  while (!Serial)
  {
    ; /* wait for serial port to connect. Needed for native USB port only */
  }
  unsigned long currTimeIMU = millis();
  if (currTimeIMU - prevTimeIMU  >= eventTimeIMU)
  {
    prevTimeIMU = currTimeIMU;
    timea[countimu] = millis(); //timeStamp;
    axa[countimu] = a.acceleration.x;
    aya[countimu] = a.acceleration.y;
    aza[countimu] = a.acceleration.z;
    gxa[countimu] = g.gyro.x;
    gya[countimu] = g.gyro.y;
    gza[countimu] = g.gyro.z;
    countimu++;
    runOnce = 0;
    //counter = 0;
    Serial.println("done.");
  }
}

void WriteArray() //This is used to prepare the SD card for reading and entering the data
{
  if (runOnce == 0) {
    File myFile = theSD.open("IMU_ARRAY.csv", FILE_WRITE);
    /* If the file opened okay, write to it */
    if (myFile) {
      Serial.print("Writing to IMU_ARRAY.csv...");
      for (int i = 0; i <= N - 1; i++) {
        myFile.print(timea[i]);
        myFile.print(",");
        myFile.print(axa[i]);
        myFile.print(",");
        myFile.print(aya[i]);
        myFile.print(",");
        myFile.print(aza[i]);
        myFile.print(",");
        myFile.print(gxa[i]);
        myFile.print(",");
        myFile.print(gya[i]);
        myFile.print(",");
        myFile.println(gza[i]);
        //myFile.println("");
      }

      //myFile.println();
      /* Close the file */
      myFile.close();
      Serial.println("done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening IMU_ARRAY.csv");
    }
    runOnce = 1;
    countimu = 0;
    FFTdone = 0;
  }
}



void setup()
{
  setupForTemp();
  setupForMPU();
  setupForGPS();
  WriteHeadings();// put your setup code here, to run once:
}

void loop()
{
  int i = 0;
  unsigned long startTime = millis();
  while (i == 0)
  {
    if ((millis() - startTime) < 10000)
    {

      ReadIMU();
    }
    else if (millis() - startTime >= 10000 && millis() - startTime < 20000)
    {

      WriteTempSat();
    }
    else
    {
      startTime = millis();
      break;
    }
  }
}
