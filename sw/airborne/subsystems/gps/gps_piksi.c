/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/gps/gps_piksi.c
 *
 * Driver for Piksi modules from Swift-Nav
 *
 * http://docs.swiftnav.com/wiki/Piksi_Integration_Tutorial
 * https://github.com/swift-nav/sbp_tutorial
 */

#include "subsystems/gps/gps_piksi.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "math/pprz_geodetic_double.h"
#if GPS_USE_LATLONG
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#endif

#include <libsbp/sbp.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/tracking.h>
#include <libsbp/settings.h>
#include <libsbp/piksi.h>

/*
 * Set the Piksi GPS antenna (default is Patch, internal)
 */
#if USE_PIKSI_EXT_ANTENNA
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""External";
#elif USE_PIKSI_AUTO_ANTENNA
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""Auto";
#else
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""Patch";
#endif

/*
 * Set the UART config depending on which UART is connected
 */
#if USE_PIKSI_UARTA
static const char SBP_UART_SET1[] = "uart_uarta""\x00""mode""\x00""SBP";
static const char SBP_UART_SET2[] = "uart_uarta""\x00""sbp_message_mask""\x00""784"; //0x310 which masks all navigation and tracking messages
static const char SBP_UART_SET3[] = "uart_uarta""\x00""configure_telemetry_radio_on_boot""\x00""False";
#else
static const char SBP_UART_SET1[] = "uart_uartb""\x00""mode""\x00""SBP";
static const char SBP_UART_SET2[] = "uart_uartb""\x00""sbp_message_mask""\x00""784"; //0x310 which masks all navigation and tracking messages
static const char SBP_UART_SET3[] = "uart_uartb""\x00""configure_telemetry_radio_on_boot""\x00""False";
#endif

/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/*
 * SBP callback nodes must be statically allocated. Each message ID / callback
 * pair must have a unique sbp_msg_callbacks_node_t associated with it.
 */
sbp_msg_callbacks_node_t pos_ecef_node;
sbp_msg_callbacks_node_t vel_ecef_node;
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t tracking_state_node;
sbp_msg_callbacks_node_t tracking_state_dep_a_node;


static void gps_piksi_publish(void);
uint32_t gps_piksi_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)));
uint32_t gps_piksi_write(uint8_t *buff, uint32_t n, void *context __attribute__((unused)));

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */
static void sbp_pos_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  static uint8_t last_flags = 0;
  msg_pos_ecef_t pos_ecef = *(msg_pos_ecef_t *)msg;

  // Check if we got RTK fix (FIXME when libsbp has a nicer way of doing this)
  if(pos_ecef.flags > 0 ){//|| last_flags == 0) {
    gps.ecef_pos.x = (int32_t)(pos_ecef.x * 100.0);
    gps.ecef_pos.y = (int32_t)(pos_ecef.y * 100.0);
    gps.ecef_pos.z = (int32_t)(pos_ecef.z * 100.0);
    gps.pacc = (uint32_t)(pos_ecef.accuracy);// FIXME not implemented yet by libswiftnav
    gps.num_sv = pos_ecef.n_sats;
    gps.tow = pos_ecef.tow;

    if(pos_ecef.flags == 1)
      gps.fix = GPS_FIX_RTK;
    else if(pos_ecef.flags == 2)
      gps.fix = GPS_FIX_DGPS;
    else
      gps.fix = GPS_FIX_3D;
  }
  last_flags = pos_ecef.flags;

  if(pos_ecef.flags > 0) gps_piksi_publish(); // Only if RTK position
}

static void sbp_vel_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  msg_vel_ecef_t vel_ecef = *(msg_vel_ecef_t *)msg;
  gps.ecef_vel.x = (int32_t)(vel_ecef.x / 10);
  gps.ecef_vel.y = (int32_t)(vel_ecef.y / 10);
  gps.ecef_vel.z = (int32_t)(vel_ecef.z / 10);
  gps.sacc = (uint32_t)(vel_ecef.accuracy);

  // Solution available (VEL_ECEF is the last message to be send)
  gps_piksi_publish(); // TODO: filter out if got RTK position
}

static void sbp_pos_llh_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  static uint8_t last_flags = 0;
  msg_pos_llh_t pos_llh = *(msg_pos_llh_t *)msg;

  // Check if we got RTK fix (FIXME when libsbp has a nicer way of doing this)
  if(pos_llh.flags > 0 || last_flags == 0) {
    gps.lla_pos.lat = (int32_t)(pos_llh.lat * 1e7);
    gps.lla_pos.lon = (int32_t)(pos_llh.lon * 1e7);
    int32_t alt = (int32_t)(pos_llh.height * 1000.);
#if GPS_USE_LATLONG
    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla_f;
    LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
    struct UtmCoor_f utm_f;
    utm_f.zone = nav_utm_zone0;
    /* convert to utm */
    utm_of_lla_f(&utm_f, &lla_f);
    /* copy results of utm conversion */
    gps.utm_pos.east = utm_f.east * 100;
    gps.utm_pos.north = utm_f.north * 100;
    gps.utm_pos.alt = gps.lla_pos.alt;
    gps.utm_pos.zone = nav_utm_zone0;
    // height is above ellipsoid or MSL according to bit flag (but not both are available)
    // 0: above ellipsoid
    // 1: above MSL
    // we have to get the HMSL from the flight plan for now
    if (bit_is_set(pos_llh.flags, 3)) {
      gps.hmsl = alt;
      gps.lla_pos.alt = alt + NAV_MSL0;
    } else {
      gps.lla_pos.alt = alt;
      gps.hmsl = alt - NAV_MSL0;
    }
#else
    // but here we fill the two alt with the same value since we don't know HMSL
    gps.lla_pos.alt = alt;
    gps.hmsl = alt;
#endif
  }
  last_flags = pos_llh.flags;
}

static void sbp_vel_ned_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  msg_vel_ned_t vel_ned = *(msg_vel_ned_t *)msg;
  gps.ned_vel.x = (int32_t)(vel_ned.n / 10);
  gps.ned_vel.y = (int32_t)(vel_ned.e / 10);
  gps.ned_vel.z = (int32_t)(vel_ned.d / 10);
#if GPS_USE_LATLONG
  gps.gspeed = int32_sqrt(gps.ned_vel.x * gps.ned_vel.x + gps.ned_vel.y * gps.ned_vel.y);
  gps.course = (int32_t)(1e7 * atan2(gps.ned_vel.y, gps.ned_vel.x));
#endif
}

static void sbp_dops_callback(uint16_t sender_id __attribute__((unused)),
                              uint8_t len __attribute__((unused)),
                              uint8_t msg[],
                              void *context __attribute__((unused)))
{
  msg_dops_t dops = *(msg_dops_t *)msg;
  gps.pdop = dops.pdop;
}

static void sbp_gps_time_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  msg_gps_time_t gps_time = *(msg_gps_time_t *)msg;
  gps.week = gps_time.wn;
  gps.tow = gps_time.tow;
}

static void sbp_tracking_state_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  uint8_t channels_cnt = len/sizeof(tracking_channel_state_t);
  msg_tracking_state_t *tracking_state = (msg_tracking_state_t *)msg;

  for(uint8_t i = 0; i < channels_cnt; i++) {
    if(tracking_state->states[i].state == 1) {
      gps.svinfos[i].svid = tracking_state->states[i].sid + 1;
      gps.svinfos[i].cno  = tracking_state->states[i].cn0;
    }
  }
}

static void sbp_tracking_state_dep_a_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  uint8_t channels_cnt = len/sizeof(tracking_channel_state_dep_a_t);
  msg_tracking_state_dep_a_t *tracking_state = (msg_tracking_state_dep_a_t *)msg;

  for(uint8_t i = 0; i < channels_cnt; i++) {
    if(tracking_state->states[i].state == 1) {
      gps.svinfos[i].svid = tracking_state->states[i].prn + 1;
      gps.svinfos[i].cno  = tracking_state->states[i].cn0;
    }
  }
}

/*
 * Initialize the Piksi GPS and write the settings
 */
void gps_impl_init(void)
{
  /* Setup SBP nodes */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_POS_ECEF, &sbp_pos_ecef_callback, NULL, &pos_ecef_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_ECEF, &sbp_vel_ecef_callback, NULL, &vel_ecef_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback, NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback, NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback, NULL, &dops_node);
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback, NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_TRACKING_STATE, &sbp_tracking_state_callback, NULL, &tracking_state_node);
  sbp_register_callback(&sbp_state, SBP_MSG_TRACKING_STATE_DEP_A, &sbp_tracking_state_dep_a_callback, NULL, &tracking_state_dep_a_node);

  /* Write settings */
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_ANT_SET), (u8*)(&SBP_ANT_SET), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET1), (u8*)(&SBP_UART_SET1), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET2), (u8*)(&SBP_UART_SET2), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET3), (u8*)(&SBP_UART_SET3), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_SAVE, SBP_SENDER_ID, 0, NULL, gps_piksi_write);
  /*msg_base_pos_t base_pos;
  base_pos.lat = 51.991152;
  base_pos.lon = 4.378052;
  base_pos.height = 50.;
  sbp_send_message(&sbp_state, SBP_MSG_BASE_POS, SBP_SENDER_ID, sizeof(msg_base_pos_t), (u8*)(&base_pos), gps_piksi_write);*/

  gps.nb_channels = GPS_NB_CHANNELS;
}

/*
 * Event handler for reading the GPS UART bytes
 */
void gps_piksi_event(void)
{
  // call sbp event function
  if (uart_char_available(&(GPS_LINK)))
    sbp_process(&sbp_state, &gps_piksi_read);
}

/*
 * Publish the GPS data from the Piksi on the ABI bus
 */
static void gps_piksi_publish(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix >= GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_PIKSI_ID, now_ts, &gps);
}

/*
 * Read bytes from the Piksi UART connection
 * This is a wrapper functions used in the libsbp library
 */
uint32_t gps_piksi_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  uint32_t i;
  for (i = 0; i < n; i++) {
    if (!uart_char_available(&(GPS_LINK)))
      break;

    buff[i] = uart_getch(&(GPS_LINK));
  }
  return i;
}

/*
 * Write bytes to the Piksi UART connection
 * This is a wrapper functions used in the libsbp library
 */
uint32_t gps_piksi_write(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  uint32_t i = 0;
  for (i = 0; i < n; i++) {
    uart_put_byte(&(GPS_LINK), buff[i]);
  }
  return n;
}

/**
 * Override the default GPS packet injector to inject the data trough UART
 */
void gps_inject_data(uint8_t packet_id, uint8_t length, uint8_t *data)
{
  sbp_send_message(&sbp_state, packet_id, SBP_SENDER_ID, length, data, gps_piksi_write);
}
