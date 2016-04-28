#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define OUTPUT_READABLE_EULER
// #define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
#define OUTPUT_TEAPOT

bool blinkState = false;

int bluetoothTx = 5;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 4;  // RX-I pin of bluetooth mate, Arduino D3
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1;           // [w, x, y, z]         quaternion container
Quaternion q2;           // [w, x, y, z]         quaternion container
VectorInt16 aa1;         // [x, y, z]            accel sensor measurements
VectorInt16 aa2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal1;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaReal2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld1;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 aaWorld2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity1;    // [x, y, z]            gravity vector
VectorFloat gravity2;    // [x, y, z]            gravity vector
float euler1[3];         // [psi, theta, phi]    Euler angle container
float euler2[3];         // [psi, theta, phi]    Euler angle container
float ypr1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket1[22] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

MPU6050 mpu1(0x69);
MPU6050 mpu2(0x68); // <-- use for AD0 high



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	Serial.begin(115200);  // Begin the serial monitor at 115200bps
	bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
	//bluetooth.print("$$$");  // Enter command mode
	Serial.println("Setting bluetooth baud rate...");
	bluetooth.println("U,115200,N");  // Temporarily Change the baudrate to 9600, no parity

	//bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
	//while (!Serial);

	//delay(50);  // Short delay, wait for the Mate to send back CMD
	//115200 can be too fast at times for NewSoftSerial to relay the data reliably
	
	bluetooth.begin(115200);  // Start bluetooth serial at 9600
	Serial.println("Initializing I2C devices...");
	bluetooth.println(F("Initializing I2C devices..."));
	
	mpu1.initialize();
	Serial.println(F("MPU1 [online]"));
	bluetooth.println(F("MPU1 [online]"));
	mpu2.initialize();
	Serial.println(F("MPU2 [online]"));
	bluetooth.println(F("MPU2 [online]"));

	Serial.println(F("Testing device connections..."));
	bluetooth.println(F("Testing device connections..."));
	
	Serial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
	Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));
	bluetooth.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
	bluetooth.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));
	
	Serial.println(F("Initializing DMP..."));
	bluetooth.println(F("Initializing DMP..."));
	
	Serial.println(F("Setting devStatus..."));
	devStatus1 = mpu1.dmpInitialize();
	devStatus1 = mpu2.dmpInitialize();

	mpu2.setXAccelOffset(-700);
	mpu2.setYAccelOffset(-1971);
	mpu2.setZAccelOffset(2506);

	mpu2.setXGyroOffset(-118);
	mpu2.setYGyroOffset(820);
	mpu2.setZGyroOffset(-425);

	mpu1.setXAccelOffset(-1320);
	mpu1.setYAccelOffset(-3470);
	mpu1.setZAccelOffset(1100);

	mpu1.setXGyroOffset(145);
	mpu1.setYGyroOffset(21);
	mpu1.setZGyroOffset(15);

	/* mpu2.setXGyroOffset(220);
	mpu2.setYGyroOffset(76);
	mpu2.setZGyroOffset(15);
	mpu2.setZAccelOffset(1688); // 1688 factory default for my test chip
	*/

	Serial.println("Pre-check for devStatus...");
	if (devStatus1 == 0 && devStatus2 == 0) {
		// Turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		bluetooth.println(F("Enabling DMP..."));
		
		mpu1.setDMPEnabled(true);
		mpu2.setDMPEnabled(true);

		// Enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		bluetooth.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus1 = mpu1.getIntStatus();
		mpuIntStatus2 = mpu2.getIntStatus();

		// Set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		bluetooth.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// Get expected DMP packet size for later comparison
		packetSize1 = mpu1.dmpGetFIFOPacketSize();
		packetSize2 = mpu2.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus1);
		Serial.print(devStatus2);
		Serial.println(F(")"));

		bluetooth.print(F("DMP Initialization failed (code "));
		bluetooth.print(devStatus1);
		bluetooth.print(devStatus2);
		bluetooth.println(F(")"));
	}

	Serial.println("Configuring activity LED...");
	// Configure LED for output
	pinMode(LED_PIN, OUTPUT);
}

void loop() {

	if (!dmpReady) {
		Serial.println("DMP loop failed -- not ready...");
		return;
	}

	while (!mpuInterrupt && fifoCount1 < packetSize1 && fifoCount2 < packetSize2 ) {}

	// Update mpuInterrupt and status flags
	Serial.println("mpuInterrupt triggered!");
	mpuInterrupt = false;
	Serial.println("mpuInterrupt reset!");

	mpuIntStatus1 = mpu1.getIntStatus();
	Serial.println("MPU1 status poll ... [OK]");

	mpuIntStatus2 = mpu2.getIntStatus();
	Serial.println("MPU2 status poll ... [OK]");
	
	Serial.println("MPU1 FIFO count poll ...");
	fifoCount1 = mpu1.getFIFOCount();
	Serial.println("MPU1 FIFO count poll ... [OK]");

	Serial.println("MPU2 FIFO count poll ...");
	fifoCount2 = mpu2.getFIFOCount();
	Serial.println("MPU2 FIFO count poll ... [OK]");

	if ((mpuIntStatus1 & 0x10) || (mpuIntStatus2 & 0x10) || fifoCount1 == 1024 || fifoCount2 == 1024) {
		// Reset so we can continue cleanly
		Serial.println(F("FIFO overflow!"));
		mpu1.resetFIFO();
		mpu2.resetFIFO();
		Serial.println(F("Both MPUs FIFO reset OK!"));
	} else if (mpuIntStatus1 & 0x02 &&  mpuIntStatus2 & 0x02 ) {
		// Wait for correct available data length, should be a VERY short wait
		Serial.println(F("Waiting for fifoCounts ..."));
		
		while (fifoCount1 < packetSize1) fifoCount1 = mpu1.getFIFOCount();
		Serial.println("fifoCount1 -- [OK]");
		
		while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();
		Serial.println("fifoCount2 -- [OK]");

		// Read packets from FIFO
		Serial.println("Reading packet from MPU1 ...");
		if (fifoCount1 <= sizeof fifoBuffer1) {
			mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
		}
		Serial.println("Reading packet from MPU1 [OK]");

		Serial.println("Reading packet from MPU2 ...");
		if (fifoCount2 <= sizeof fifoBuffer2) {
			mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
		}
		Serial.println("Reading packet from MPU2 [OK]");

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		Serial.println("Retracking fifoCount1 ...");
		fifoCount1 -= packetSize1;

		Serial.println("Retracking fifoCount2 ...");
		fifoCount2 -= packetSize2;

		#ifdef OUTPUT_READABLE_QUATERNION
			// display quaternion values in easy matrix form: w x y z
			mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
			bluetooth.print("quat\t");
			bluetooth.print(q2.w);
			bluetooth.print("\t");
			bluetooth.print(q2.x);
			bluetooth.print("\t");
			bluetooth.print(q2.y);
			bluetooth.print("\t");
			bluetooth.println(q2.z);
		#endif

		#ifdef OUTPUT_READABLE_EULER
			// display Euler angles in degrees
			mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
			mpu1.dmpGetEuler(euler1, &q1);
			mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
			mpu2.dmpGetEuler(euler2, &q2);
			//         Serial.print("#");

			//  bluetooth.print("euler1\t");
			//  bluetooth.print(euler1[0] * 180/M_PI);
			//         bluetooth.print("+");
			bluetooth.print("\t");
			// bluetooth.print(euler1[1] * 180/M_PI);
			//            bluetooth.print("+");
			// bluetooth.print("\t");
			// bluetooth.println(euler1[2] * 180/M_PI);
			//          bluetooth.print("+");
			bluetooth.print("euler2\t");
			bluetooth.print(euler2[0] * 180 / M_PI);
			//   bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.print(euler2[1] * 180 / M_PI);
			// bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.println(euler2[2] * 180 / M_PI);
			//        bluetooth.print("+");
		#endif

		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees
			mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
			mpu1.dmpGetGravity(&gravity1, &q1);
			mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
			//bluetooth.print("ypr\t");
			bluetooth.print(ypr1[0] * 180 / M_PI);
			bluetooth.print("+");
			//bluetooth.print("\t");
			bluetooth.print(ypr1[1] * 180 / M_PI);
			bluetooth.print("+");
			//bluetooth.print("\t");
			bluetooth.print(ypr1[2] * 180 / M_PI);
			bluetooth.print("+");
		#endif

		#ifdef OUTPUT_READABLE_REALACCEL
			// display real acceleration, adjusted to remove gravity
			mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
			mpu1.dmpGetAccel(&aa1, fifoBuffer1);
			mpu1.dmpGetGravity(&gravity1, &q1);
			mpu1.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
			//bluetooth.print("areal\t");
			bluetooth.print(aaReal1.x);
			//bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.print(aaReal1.y);
			//bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.println(aaReal1.z);
			//bluetooth.println("@");
		#endif

		#ifdef OUTPUT_READABLE_WORLDACCEL
		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
			mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
			mpu1.dmpGetAccel(&aa1, fifoBuffer1);
			mpu1.dmpGetGravity(&gravity1, &q1);
			mpu1.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
			mpu1.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

			mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
			mpu2.dmpGetAccel(&aa2, fifoBuffer2);
			mpu2.dmpGetGravity(&gravity2, &q2);
			mpu2.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
			mpu2.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

			bluetooth.print("aworld1\t");
			bluetooth.print(aaWorld1.x);
			//  bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.print(aaWorld1.y);
			//  bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.println(aaWorld1.z);
			//    bluetooth.print("+");
			bluetooth.print("aworld2\t");
			bluetooth.print(aaWorld2.x);
			//  bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.print(aaWorld2.y);
	//                       bluetooth.print("+");
			bluetooth.print("\t");
			bluetooth.println(aaWorld2.z);
			// bluetooth.println("@");
		#endif

		#ifdef OUTPUT_TEAPOT
		// display quaternion values in InvenSense Teapot demo format:
			teapotPacket1[2] = fifoBuffer1[0];
			teapotPacket1[3] = fifoBuffer1[1];
			teapotPacket1[4] = fifoBuffer1[4];
			teapotPacket1[5] = fifoBuffer1[5];
			teapotPacket1[6] = fifoBuffer1[8];
			teapotPacket1[7] = fifoBuffer1[9];
			teapotPacket1[8] = fifoBuffer1[12];
			teapotPacket1[9] = fifoBuffer1[13];

			teapotPacket1[10] = fifoBuffer2[0];
			teapotPacket1[11] = fifoBuffer2[1];
			teapotPacket1[12] = fifoBuffer2[4];
			teapotPacket1[13] = fifoBuffer2[5];
			teapotPacket1[14] = fifoBuffer2[8];
			teapotPacket1[15] = fifoBuffer2[9];
			teapotPacket1[16] = fifoBuffer2[12];
			teapotPacket1[17] = fifoBuffer2[13];

			bluetooth.write(teapotPacket1, 22);
			teapotPacket1[19]++; // packetCount, loops at 0xFF on purpose
		#endif

		mpu1.resetFIFO();
		mpu2.resetFIFO();
		bluetooth.flush();

		delay(10);

// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}
