// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "ubx.h"
#include "fit.h"
#include "gps_service.h"

#define TAG "logging_service"

static void logging_service_fn(void *param);

struct service *logging_service_register()
{
	return service_register("logging", logging_service_fn, 1, 4096);
}

// http://howardhinnant.github.io/date_algorithms.html#days_from_civil
// Returns number of days since civil 1970-01-01.  Negative values indicate
//    days prior to 1970-01-01.
// Preconditions:  y-m-d represents a date in the civil (Gregorian) calendar
//                 m is in [1, 12]
//                 d is in [1, last_day_of_month(y, m)]
//                 y is "approximately" in
//                   [numeric_limits<Int>::min()/366, numeric_limits<Int>::max()/366]
//                 Exact range of validity is:
//                 [civil_from_days(numeric_limits<Int>::min()),
//                  civil_from_days(numeric_limits<Int>::max()-719468)]
int days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y-399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);      // [0, 399]
    const unsigned doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;  // [0, 365]
    const unsigned doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096]
    return era * 146097 + (int)(doe) - 719468;
}

static uint32_t fit_timestamp(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
	static uint32_t epoch = 0;
	if (epoch == 0) {
		epoch = days_from_civil(1989, 12, 31);
	}
	uint32_t today = days_from_civil(year, month, day);

	return ((today - epoch) * 3600 * 24) + (hour * 3600) + (min * 60) + sec;
}

static void fit_start_file(struct fit_file *fit, uint32_t timestamp)
{
	int ret;

	{
		FIT_UINT8 local_mesg_number = 2;
		FIT_FILE_ID_MESG file_id;
		fit_init_message(FIT_MESG_FILE_ID, &file_id);
		file_id.type = FIT_FILE_ACTIVITY;
		file_id.manufacturer = FIT_MANUFACTURER_DEVELOPMENT;
		file_id.product = 0xb33f;
		file_id.serial_number = 1;
		file_id.time_created = timestamp;

		ret = fit_register_message(fit, local_mesg_number, FIT_MESG_FILE_ID);
		if (ret) {
			fprintf(stderr, "fit_register_message failed\n");
			return;
		}
		ret = fit_write_message(fit, local_mesg_number, &file_id);
		if (ret) {
			fprintf(stderr, "fit_write_message failed\n");
			return;
		}
	}

	{
		FIT_UINT8 local_mesg_number = 0;
		ret = fit_register_message(fit, local_mesg_number, FIT_MESG_ACTIVITY);
		if (ret) {
			fprintf(stderr, "fit_register_message failed\n");
			return;
		}
		FIT_ACTIVITY_MESG activity;
		fit_init_message(FIT_MESG_ACTIVITY, &activity);
		activity.timestamp = timestamp;
		activity.num_sessions = 1;
		activity.type = FIT_ACTIVITY_MANUAL;
		activity.event = FIT_EVENT_TIMER;
		activity.event_type = FIT_EVENT_TYPE_START;

		ret = fit_write_message(fit, local_mesg_number, &activity);
		if (ret) {
			fprintf(stderr, "fit_write_message failed\n");
			return;
		}
	}
}

static void logging_service_fn(void *param)
{
	struct service *service = (struct service *)param;
	struct service *gps_service = service_lookup("gps");
	int ret;

	assert(gps_service);

	enum state {
		STOPPED = 0,
		STARTING = 1,
		LOGGING = 2,
		PAUSED = 3,
	} state = STOPPED;

	struct fit_file *fit = NULL;

	FIT_SESSION_MESG session;
	fit_init_message(FIT_MESG_SESSION, &session);
	session.event = FIT_EVENT_SESSION;
	session.event_type = FIT_EVENT_TYPE_STOP;
	session.sport = FIT_SPORT_CYCLING;

	FIT_LAP_MESG lap;
	fit_init_message(FIT_MESG_LAP, &lap);
	lap.message_index = 0;
	lap.event = FIT_EVENT_LAP;
	lap.event_type = FIT_EVENT_TYPE_STOP;

	struct pvt_message *last_pvt = NULL;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_START:
			if (state == STOPPED) {
				gps_subscribe_pvt(gps_service, service);
				state = STARTING;
			}
			break;
		case SERVICE_CMD_STOP:
			if (state == LOGGING) {
				// Finalise
				ESP_LOGI(TAG, "logging finalise.");

				uint32_t timestamp = fit_timestamp(last_pvt->body->year, last_pvt->body->month, last_pvt->body->day,
								   last_pvt->body->hour, last_pvt->body->min, last_pvt->body->sec);
				int32_t lat = ubx_deg_to_semicircles(last_pvt->body->lat);
				int32_t lon = ubx_deg_to_semicircles(last_pvt->body->lon);

				{
					FIT_UINT8 local_mesg_number = 1;
					session.timestamp = timestamp;
					session.total_elapsed_time = timestamp - session.start_time;

					ret = fit_register_message(fit, local_mesg_number, FIT_MESG_SESSION);
					if (ret) {
						fprintf(stderr, "fit_register_message failed\n");
					}
					ret = fit_write_message(fit, local_mesg_number, &session);
					if (ret) {
						fprintf(stderr, "fit_write_message failed\n");
					}
				}

				{
					FIT_UINT8 local_mesg_number = 2;
					lap.timestamp = timestamp;
					lap.end_position_lat = lat;
					lap.end_position_long = lon;
					lap.total_elapsed_time = timestamp - lap.start_time;
					lap.message_index = 0;

					ret = fit_register_message(fit, local_mesg_number, FIT_MESG_LAP);
					if (ret) {
						fprintf(stderr, "fit_register_message failed\n");
					}
					ret = fit_write_message(fit, local_mesg_number, &lap);
					if (ret) {
						fprintf(stderr, "fit_write_message failed\n");
					}
				}

				ret = fit_finalise(fit);
				if (ret) {
					fprintf(stderr, "fit_finalise failed\n");
				}

				fit_destroy(fit);
				fit = NULL;
			}

			if (state != STOPPED) {
				ESP_LOGI(TAG, "logging stop.");
				gps_unsubscribe_pvt(gps_service, service);
				state = STOPPED;

				if (last_pvt) {
					pvt_put(last_pvt);
					last_pvt = NULL;
				}
			}
			break;
		case SERVICE_CMD_PAUSE:
			// Pause
			// Unsubscribe PVT
			break;
		case SERVICE_CMD_RESUME:
			// Resume
			// Subscribe PVT
			break;
		case GPS_CMD_PVT:
		{
			if (last_pvt) {
				pvt_put(last_pvt);
				last_pvt = NULL;
			}

			struct pvt_message *pvt = (struct pvt_message *)smsg.argp;
			last_pvt = pvt;

			if (state == STARTING) {
				// Start file
				state = LOGGING;
				ESP_LOGI(TAG, "starting log file: %04d-%02d-%02d %02d:%02d:%02d\n",
					 pvt->body->year, pvt->body->month, pvt->body->day, pvt->body->hour, pvt->body->min, pvt->body->sec);
				char filename[] = "/spiffs/2020-07-12_121003.fit                    ";
				sprintf(filename, "/spiffs/%04d-%02d-%02d_%02d%02d%02d.fit", 
				        pvt->body->year, pvt->body->month, pvt->body->day, pvt->body->hour, pvt->body->min, pvt->body->sec);

				uint32_t timestamp = fit_timestamp(pvt->body->year, pvt->body->month, pvt->body->day,
								   pvt->body->hour, pvt->body->min, pvt->body->sec);
				fit = fit_create(filename, timestamp, FIT_FILE_ACTIVITY);
				if (fit) {
					fit_start_file(fit, timestamp);
				}

				int32_t lat = ubx_deg_to_semicircles(pvt->body->lat);
				int32_t lon = ubx_deg_to_semicircles(pvt->body->lon);

				session.start_time = timestamp;
				session.start_position_lat = lat;
				session.start_position_long = lon;

				lap.start_time = timestamp;
				lap.start_position_lat = lat;
				lap.start_position_long = lon;

				ret = fit_register_message(fit, 0, FIT_MESG_RECORD);
				if (ret) {
					fprintf(stderr, "fit_register_message failed\n");
				}
			} else if (state == LOGGING) {
				// Write PVT message
				ESP_LOGI(TAG, "write message: %04d-%02d-%02d %02d:%02d:%02d\n",
					 pvt->body->year, pvt->body->month, pvt->body->day, pvt->body->hour, pvt->body->min, pvt->body->sec);

				{
					FIT_RECORD_MESG record;
					fit_init_message(FIT_MESG_RECORD, &record);
					uint32_t timestamp = fit_timestamp(pvt->body->year, pvt->body->month, pvt->body->day,
									   pvt->body->hour, pvt->body->min, pvt->body->sec);

					int32_t lat = ubx_deg_to_semicircles(pvt->body->lat);
					int32_t lon = ubx_deg_to_semicircles(pvt->body->lon);

					record.timestamp = timestamp;
					record.position_lat = lat;
					record.position_long = lon;
					record.gps_accuracy = pvt->body->hAcc / 1000; // Metres

					ret = fit_write_message(fit, 0, &record);
					if (ret) {
						fprintf(stderr, "fit_write_message failed\n");
					}
				}
			}
			break;
		}
		default:
			// Unknown command
			break;
		}

		// Acknowledge the command
		service_ack(service);
	}
}
