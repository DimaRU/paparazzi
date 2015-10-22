/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2002  Chris efstathiou hendrix@otenet.gr
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/sonar/sonar_srf08.c
 * @brief Basic library for SRF08 sonar
 *
 */

#include "mcu_periph/i2c.h"
#include "sonar_srf08.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"

#ifdef SITL
#include "state.h"
#endif

#ifndef SRF08_I2C_DEV
#define SRF08_I2C_DEV i2c1
#endif

/* Global Variables */
#ifndef SITL
struct i2c_transaction srf_trans;
bool_t srf08_received, srf08_got;
#endif
float srf08_distance;
uint16_t srf08_meas = 0;          // Dummy Raw ADC value


/*###########################################################################*/

void srf08_init(void)
{
#ifndef SITL
  srf08_received = FALSE;
  srf08_got = FALSE;

  /** Setting the gain to the minimun value (to avoid echos ?) */
  srf_trans.buf[0] = SRF08_SET_GAIN;
  srf_trans.buf[1] = SRF08_MIN_GAIN;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);
  
  srf_trans.buf[0] = SRF08_SET_RANGE;
  srf_trans.buf[1] = SRF08_MAX_RANGE;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);

  srf_trans.buf[0] = SRF08_COMMAND;
  srf_trans.buf[1] = SRF08_CENTIMETERS;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);

#endif
}
/*###########################################################################*/

void srf08_initiate_ranging(void)
{
#ifndef SITL
  srf_trans.buf[0] = SRF08_COMMAND;
  srf_trans.buf[1] = SRF08_CENTIMETERS;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);
#endif
}

/** Ask the value to the device */
void srf08_receive(void)
{
#ifndef SITL
  srf_trans.buf[0] = SRF08_ECHO_1;
  srf08_received = TRUE;
  i2c_transmit(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 1);
#else // SITL
  srf08_distance = stateGetPositionEnu_f()->z;
  Bound(srf08_distance, 0.1f, 6.0f);
  AbiSendMsgAGL(AGL_SONAR_ADC_ID, srf08_distance);

#ifdef SENSOR_SYNC_SEND_SONAR
  // Send Telemetry report
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &srf08_meas, &srf08_distance);
#endif
#endif // SITL
}

void srf08_event(void)
{
#ifndef SITL    
  /** Handling of data sent by the device (initiated by srf08_receive() */
  if (srf_trans.status == I2CTransSuccess) {
    if (srf08_received) {
      srf08_received = FALSE;
      srf08_got = TRUE;
      i2c_receive(&SRF08_I2C_DEV, &srf_trans, SRF08_UNIT_0, 2);
    } else if (srf08_got) {
      srf08_got = FALSE;
      srf08_distance = (float)(srf_trans.buf[0] << 8 | srf_trans.buf[1]) / 100;

      AbiSendMsgAGL(AGL_SONAR_ADC_ID, srf08_distance);

#ifdef SENSOR_SYNC_SEND_SONAR
      // Send Telemetry report
      DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &srf08_meas, &srf08_distance);
#endif
#endif // SITL
    }
  }
}
