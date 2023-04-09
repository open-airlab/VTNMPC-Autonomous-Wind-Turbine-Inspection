/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file esc_status.msg */


#include <cinttypes>
#include <cstdio>
#include <px4_defines.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>

constexpr char __orb_esc_status_fields[] = "uint64_t timestamp;uint16_t counter;uint8_t esc_count;uint8_t esc_connectiontype;uint8_t[4] _padding0;esc_report[8] esc;";

ORB_DEFINE(esc_status, struct esc_status_s, 400, __orb_esc_status_fields);


void print_message(const esc_status_s& message)
{
	printf(" esc_status_s\n");
	printf("\ttimestamp: %" PRIu64, message.timestamp);
	if (message.timestamp != 0) {
		printf(" (%.6f seconds ago)\n", hrt_elapsed_time(&message.timestamp) / 1e6);
	} else {
		printf("\n");
	}
	printf("\tcounter: %u\n", message.counter);
	printf("\tesc_count: %u\n", message.esc_count);
	printf("\tesc_connectiontype: %u\n", message.esc_connectiontype);
		printf("\tpx4/esc_report[8] esc[0]");
 print_message(message.esc[0]);
printf("\tpx4/esc_report[8] esc[1]");
 print_message(message.esc[1]);
printf("\tpx4/esc_report[8] esc[2]");
 print_message(message.esc[2]);
printf("\tpx4/esc_report[8] esc[3]");
 print_message(message.esc[3]);
printf("\tpx4/esc_report[8] esc[4]");
 print_message(message.esc[4]);
printf("\tpx4/esc_report[8] esc[5]");
 print_message(message.esc[5]);
printf("\tpx4/esc_report[8] esc[6]");
 print_message(message.esc[6]);
printf("\tpx4/esc_report[8] esc[7]");
 print_message(message.esc[7]);
}