/* Copyright (c) 2019  Paulo Costa
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
  * Neither the name of the copyright holders nor the names of
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE. */


#include "IRLine.h"
#include "Arduino.h"
#include <string.h>
#include "config.h"
#include "robot.h"

//#define DEBUG
//#define Raspberry 
#define Arduino 

/**
* @file IRLine.cpp
* @brief Implements infrared line sensor handling and line detection logic for the robot.
*
* This module manages all operations related to the infrared (IR) line sensors, including:
*  - Reading sensor values through an analog multiplexer
*  - Calculating left and right line edges
*  - Encoding sensor data into a compact format
*  - Detecting line patterns and junction nodes (left, right, T, cross, etc.)
*
* It supports both Arduino and Raspberry Pi Pico platforms through conditional compilation.
* The functions use threshold-based analysis to interpret the reflectance readings and identify
* navigation cues for line-following or maze-solving behaviors.
*
* @note Requires `robot.h`, `config.h`, and the appropriate hardware configuration for the MUX pins.
*
*/


extern robot_t robot;

/**
* @brief Construct a new IRLine_t object and initialize defaults.
*
* Initializes calibration and detection thresholds and clears working
* variables. These defaults are conservative starting values and may be
* adjusted later by a call to `calibrate()`.
*
* Defaults set in the implementation:
*  - `IR_WaterLevel = 0`
*  - `IR_tresh = 700` (threshold to classify black/white)
*  - `cross_tresh = 3` (threshold for crossing detection)
*  - `black_cross_level = 2.8` (calibration level marker)
*/
IRLine_t::IRLine_t()
{
IR_WaterLevel = 0; // baseline offset (water level) subtracted from raw readings
IR_tresh = 500; // threshold used to decide black vs white on a 10-bit ADC (0..1023)
cross_tresh = 3; // crossing detection threshold (count of sensors/signals)
black_cross_level = 2.8; // calibration marker for a black surface (project-specific units)
}

/**
* @brief Calibrate the IR sensors.
*
* Intended to sample sensors under known black/white surfaces and adjust
* internal thresholds (for example `IR_tresh` and `IR_max`).
*
* @note Currently this function is a stub (not implemented).
*/
void IRLine_t::calibrate(void)
{

}

/**
* @brief Compute the left line edge using the left-to-right sensor scan.
*
* The algorithm scans the five IR sensors, subtracts `IR_WaterLevel` from
* each reading, clamps negatives to zero, and searches for the first
* threshold crossing where the previous sample is below `IR_tresh` and the
* current sample is above `IR_tresh`. When a crossing is found, an
* interpolated position is computed and stored in `pos_left`.
*
* Side effects:
*  - updates `pos_left`, `total` (sum of adjusted readings) and `IR_max`.
*/
void IRLine_t::calcIRLineEdgeLeft(void)
{
byte c; // number of IR sensor
byte found; // flag that tell detect the left edge
int v; // Current adjusted IR sensor reading.
int last_v;// Lasted adjusted IR sensor reading

found = 0;
IR_max = 0;
pos_left = 2 * 16.0;
total = 0; // accumulate IR signal strength (not used)
last_v = 0;
for (c = 0; c < 5; c++) {
v = IR_values[c] - IR_WaterLevel;

if (v < 0) v = 0;
if (v > IR_max) IR_max = v;

total = total + v;

if (!found && last_v < IR_tresh && v > IR_tresh) {
pos_left = -12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v);
found = 1; // Found the edge
}
last_v = v;
}
}

/**
* @brief Compute the right line edge by scanning sensors right-to-left.
*
* Same algorithm as `calcIRLineEdgeLeft()` but scanning from the outer
* right sensor towards the left. Results are stored in `pos_right`.
*
* Side effects:
*  - updates `pos_right`, `total` and `IR_max`.
*/
void IRLine_t::calcIRLineEdgeRight(void)
{
// this is the same the leftedge
byte c, found;
int v, last_v;

found = 0;
IR_max = 0;
pos_right = -2 * 16.0;
total = 0;
last_v = 0;
for (c = 0; c < 5; c++) {
v = IR_values[4 - c] - IR_WaterLevel;
if (v < 0) v = 0;
if (v > IR_max) IR_max = v;
total = total + v;

if (!found && last_v < IR_tresh && v > IR_tresh) {
pos_right = -(-12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v));
found = 1;
}
last_v = v;
}

}


/**
* @brief Set the external analog multiplexer input channel.
*
* Writes the appropriate binary values to the three MUX control pins (A,B,C)
* so the external multiplexer connects the desired channel to the ADC pin.
* Platform-specific GPIO calls are used under conditional compilation.
*
* @param channel MUX channel number (typically 0..7).
*/
static void adc_set_channel(int channel) 
{
#ifdef Raspberry
gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | digitalPinToBitMask(MUXB_PIN) | digitalPinToBitMask(MUXC_PIN), channel << MUXA_PIN);// This function (from the Raspberry Pi Pico SDK) sets several GPIO pins at once using a bitmask.
#endif
#ifdef Arduino
digitalWrite(MUXA_PIN, channel & 1);
digitalWrite(MUXB_PIN, (channel >> 1) & 1);
digitalWrite(MUXC_PIN, (channel >> 2) & 1);

#endif
}

/**
* @brief Read a single ADC sample from the specified MUX channel.
*
* The function sets the MUX channel, waits a short time for the signal to
* settle, and then reads the value from analog input A2 where the MUX is
* connected.
*
* @param channel MUX channel number to read.
* @return uint16_t ADC reading, range 0..1023 (10-bit ADC).
* @note A delayMicroseconds(100) is used to allow the multiplexer output to
*       settle before sampling.
*/
uint16_t read_adc(int channel)
{
adc_set_channel(channel); // Swi  tch external MUX to the desired channel
delayMicroseconds(100);
return analogRead(A2);    // The mux connects to analog input A2
}

/**
* @brief Read all IR sensors via the analog multiplexer and store values.
*
* The code reads MUX channels 3..7, inverts the ADC value (1023 - raw) and
* stores the results into the robot's IRLine array in reversed order. The
* inversion converts reflectance to a convention where higher values mean
* darker (more reflective to IR) and the ordering matches the rest of the
* codebase.
*
* Side effects: writes into `robot.IRLine.IR_values[]`.
*/
void IRLine_t::readIRSensors(void)
{
byte c;  // Read the five IR sensors using the AD converter
for (c = 0; c < IRSENSORS_COUNT; c++) 
{
robot.IRLine.IR_values[(IRSENSORS_COUNT - 1) -c] = 1023 - read_adc(3 + c);
}
//Serial.println();
}

/**
* @brief Encode the five IR sensor readings into a compact 32-bit value.
*
* Each sensor reading is a 10-bit value (0..1023). This function reduces
* each reading to 6 bits by right-shifting 4 bits (lossy compression) and
* packs the five 6-bit values into a 32-bit integer (30 used bits).
*
* Bit layout (from most to least significant bits):
*  - bits [29:24] = IR_values[0] >> 4
*  - bits [23:18] = IR_values[1] >> 4
*  - bits [17:12] = IR_values[2] >> 4
*  - bits [11:6]  = IR_values[3] >> 4
*  - bits [5:0]   = IR_values[4] >> 4
*
* @return uint32_t packed sensor state (0..(1<<30)-1), useful for logging or
*         compact comparisons.
*/
uint32_t IRLine_t::encodeIRSensors(void)
{
byte c;                                           // Encode five IR sensors with 6 bits for each sensor
uint32_t result = robot.IRLine.IR_values[0] >> 4; // From 10 bits to 6 bits
for (c = 1; c < 5; c++)
{
result = (result << 6) | (robot.IRLine.IR_values[c] >> 4);
}
return result;
}






/**
* @brief Print the current IR sensor values to Serial for debugging.
*
* @note Serial must be initialized by the caller (Serial.begin) before
*       calling this function.
*/
void IRLine_t::printIRLine(void)
{
Serial.print(robot.IRLine.IR_values[0]);
Serial.print("               ");
Serial.print(robot.IRLine.IR_values[1]);
Serial.print("               ");
Serial.print(robot.IRLine.IR_values[2]);
Serial.print("               ");
Serial.print(robot.IRLine.IR_values[3]);
Serial.print("               ");
Serial.println(robot.IRLine.IR_values[4]);
}



/**
* Node detection
* @brief Analyze the five IR sensors and classify the current "node" (road pattern).
*
* last_node mapping:
*  - E -> Error or unknown
*  - L -> Left junction
*  - R -> Right junction
*  - B -> Both sides black (cross or T-junction)
*  - W -> White (no line, maybe after a junction or U-turn)
*  - N -> Normal line
*
| Condition          | Pattern                        | Meaning                              | Returned char |
| ------------------ | ------------------------------ | ------------------------------------ | ------------- |
| `XXXOO`            | Left 3 dark, right 2 light     | Left node detected                   | `'L'`         |
| `OOXXX`            | Right 3 dark, left 2 light     | Right node detected                  | `'R'`         |
| `XXXXX`            | All dark                       | Cross / T-junction / End             | `'B'`         |
| `OOOOO`            | All light                      | Off the line (after turn or no line) | `'W'`         |
| Single sensor dark | Regular line following pattern | `'N'`                                |               |
| Anything else      | No clear match                 | `'E'`                                |               |

* @return char Node code as described above. Returns 'E' until a stable node
*              is observed for the required number of cycles.
*/
char IRLine_t::detectNode( )
{
static char last_node = 'E';
static int stability_count = 0;
static int  required_stability = 2;

char binary_pattern[6] = "";  // Initialize empty
for(int i = 0; i < 5; i++) {
    char bit[2] = { (IR_values[i] > IR_tresh ? '1' : '0'), '\0' };  // Single char as string
    strcat(binary_pattern, bit);  // Append the bit
} 

char current_node;

if(strcmp(binary_pattern, "11100") == 0 || strcmp(binary_pattern, "11000") == 0) {
    current_node = 'L'; // Left node detected
} else if (strcmp(binary_pattern, "00111") == 0 || strcmp(binary_pattern, "00011") == 0) {
    current_node = 'R'; // Right node detected
} else if (strcmp(binary_pattern, "11111") == 0) {
    current_node = 'B'; // Cross / T-junction / End
} else if (strcmp(binary_pattern, "00000") == 0) {
    current_node = 'W'; // Off the line
} else if (strcmp(binary_pattern, "01110") == 0 || strcmp(binary_pattern, "01100") == 0 ||
           strcmp(binary_pattern, "00110") == 0) {
    current_node = 'N'; // Normal line
} else {
    current_node = 'N'; // Error or unknown
}


// If the same node type is detected again → increase the stability counter.
// If it changes → reset the counter and update last_node.

if (current_node == last_node)
{
stability_count++;
}
else
{
stability_count = 0;
last_node = current_node;
}

#ifdef DEBUG
Serial.print("Node: ");
Serial.print(current_node);
Serial.println();
#endif
return current_node;

}