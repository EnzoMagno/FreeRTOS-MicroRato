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

#ifndef IRLINE_H
#define IRLINE_H

#include "Arduino.h"
#include <string.h>

#define IRSENSORS_COUNT 5 //Count of IR sensors

/**
 * @file IRLine.h
 * @brief Interface for reading and processing a line of IR sensors used by the
 * micro-rato robot for line following and node detection.
 *
 * This header declares the IRLine_t class which stores the raw sensor values
 * and provides helpers to read sensors, encode the sensor state, compute line
 * edges, calibrate sensors and detect nodes (crossings). 
 *
 * Usage summary:
 *  - call readIRSensors() periodically to sample the IRSensors into
 *    `IR_values`;
 *  - use calcIRLineEdgeLeft()/calcIRLineEdgeRight() to compute a line position
 *    estimate; and
 *  - call detectNode() to check for nodes/crossings on the line.
 *
 */
class IRLine_t
{
  public:
    /**
     * @brief Position estimate contributed by left/rigth/total sensor.
     *
     * Value is a floating point position computed by the edge/centroid
     * algorithms. Interpretation is project-specific (higher means more to
     * the right/left depending on the algorithm used).
     */
    float pos_left, pos_right, total;

    /**
     * @brief Raw ADC readings from the line IR sensors.
     *
     * The array length is IRSENSORS_COUNT and each element holds the latest
     * sample for that sensor. Values are typically compared against
     * `IR_tresh` to determine black/white.
     */
    int IR_values[IRSENSORS_COUNT];

    /** @brief Save the IR data in the notation like the water level. */
    int IR_WaterLevel;

    /**
     * @brief Threshold used to decide if a sensor sees a black surface.
     *
     * Default initialized to 850 in the implementation. Values greater than
     * this threshold typically indicate a black (line) reading on the ADC.
     */
    int IR_tresh = 850, IR_max;

    /** @brief Number of sensor crossings currently detected (internal use). */
    int crosses;

    /** @brief Counts used to track node crossings between samples. */
    int cross_count, last_cross_count;

    /** @brief Threshold for deciding when a crossing is present. */
    int cross_tresh;

    /** @brief ADC level considered a black cross for node detection. */
    int black_cross_level;

    /** @brief Flag that tell if is detecting black. */
    float blacks;

    /**
     * @brief Construct a new IRLine_t instance.
     *
     * Initializes internal counters and calibration values. See
     * implementation in IRLine.cpp for defaults.
     */
    IRLine_t();

    /**
     * @brief Read all IR sensors and store results in `IR_values`.
     *
     * Samples the underlying ADC inputs for each IR sensor and
     * updates internal state used by other helpers.
     */
    void readIRSensors(void);

    /**
    * @brief  This function compresses them to 6 bits each and stores all five inside one 32-bit number
    *You have 5 IR sensors, each giving a 10-bit reading (from analogRead(), range 0–1023).
    * @return uint32_t bitmask representing sensor states
    */
    uint32_t encodeIRSensors(void);

    /**
     * @brief Print the current IR state over Serial for debugging.
     *
     * Prints raw sensor values, thresholds and computed positions. Implementation
     * uses Arduino Serial; calling code should ensure Serial is initialized.
     */
    void printIRLine(void);
    
    /**
     * @brief Calibrate sensors by sampling background and line values.
     *
     * Uses internal averaging or min/max logic to set `IR_tresh` and `IR_max`.
     * Call this at startup or when lighting conditions change.
     */
    void calibrate(void);
    
    /**
     * @brief Compute line edge/position using the left set of sensors.
     *
     * The result is stored in `pos_left` and `total` and can be used by the
     * steering controller to follow the line.
     */
    void calcIRLineEdgeLeft(void);

    /**
     * @brief Compute line edge/position using the right set of sensors.
     *
     * The result is stored in `pos_right` and `total` and can be used by the
     * steering controller to follow the line.
     */
    void calcIRLineEdgeRight(void);
    //void calcIRLineCenter(void);
    

    /**
     * @brief Detect a node (crossing) and return a character code.
     *
     * The character return value is project-specific (for example 'L', 'R',
     * 'T', '.' for none). See implementation for mapping details.
     *
     * @return char node code or '.' when no node is detected
     */
    char detectNode();

};

#endif // IRLINE_H
