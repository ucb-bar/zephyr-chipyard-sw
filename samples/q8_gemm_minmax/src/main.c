/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <pthreadpool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/_intsup.h>
#include <xnnpack.h> // Include XNNPack headers
#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include "dim.h"  // Include dimension definitions

static int8_t input_data[BATCH_SIZE * INPUT_CHANNELS];
static int8_t weights[INPUT_CHANNELS * OUTPUT_CHANNELS];
static float scale[OUTPUT_CHANNELS];
static int32_t bias[OUTPUT_CHANNELS];
static int8_t output_data[BATCH_SIZE * OUTPUT_CHANNELS];
static int8_t output_data_ref[BATCH_SIZE * OUTPUT_CHANNELS];

unsigned long cycle()
{
	unsigned long cc;
	__asm__ volatile("rdcycle  %0" : "=r"(cc));
	return cc;
}

int main(void)
{
	printf("Target: %s\n", CONFIG_BOARD_TARGET);
	printf("CPUs: %d\n", CONFIG_MP_MAX_NUM_CPUS);
	printf("XNNPACK QS8\n");

	// Test malloc
	void *test_alloc = aligned_alloc(0x40, 0x380);
	test_alloc = aligned_alloc(0x40, 0x380);
	if (!test_alloc) {
		printf("Test malloc failed!\n");
		return -1;
	} else {
		printf("Test malloc succeeded!\n");
		free(test_alloc);
	}

	// create pthreadpool
	pthreadpool_t threadpool = NULL;
	threadpool = pthreadpool_create(0);

	if (threadpool == NULL) {
		printf("Failed to create pthreadpool\n");
		return -1;
	}

	// Initialize XNNPACK
	int status = xnn_initialize(NULL);
	if (status != xnn_status_success) {
		printf("Failed to initialize XNNPack, status code: %d\n", status);
		return -1;
	}

	printf("Creating operators\n");

	// int8_t *input_data = (int8_t *)malloc(batch_size * input_channels * sizeof(int8_t));
	// int8_t *weights = (int8_t *)malloc(input_channels * output_channels * sizeof(int8_t));
	// float *scale = (float *)malloc(output_channels * sizeof(float));
	// int32_t *bias = (int32_t *)malloc(output_channels * sizeof(int32_t));
	// int8_t *output_data = (int8_t *)malloc(batch_size * output_channels * sizeof(int8_t));
	// int8_t *output_data_ref = (int8_t *)malloc(batch_size * output_channels * sizeof(int8_t));
	
	int8_t minzp = -128;
	int8_t maxzp = 127;
	int8_t outzp = 0;

	printf("chIn: %zu, chOut: %zu, bSz: %zu\n", (size_t)INPUT_CHANNELS, (size_t)OUTPUT_CHANNELS, (size_t)BATCH_SIZE);

	// Initialize input data
	printf("Initializing input data\n");
	int8_t zero_point = 0;
	for (size_t i = 0; i < BATCH_SIZE * INPUT_CHANNELS; i++) {
		input_data[i] = (int8_t)(1);
		// input_data[i] = (int8_t)(i - (BATCH_SIZE*INPUT_CHANNELS>>1));
	}
	printf("Initializing weights\n");
	// Initialize weights
	for (size_t i = 0; i < INPUT_CHANNELS * OUTPUT_CHANNELS; i++) {
		// weights[i] = (int8_t)((i - ((INPUT_CHANNELS*OUTPUT_CHANNELS)>>1)));
		weights[i] = (int8_t)(1);
	}
	printf("Initializing scale, bias\n");

	for (size_t i = 0; i < OUTPUT_CHANNELS; i++) {
		scale[i] = (float)1.0f;
		bias[i] = (int32_t)(i - (OUTPUT_CHANNELS>>1));
	}
	printf("Computing reference output\n");
	// Compute reference output
	for (size_t b = 0; b < BATCH_SIZE; b++) {
		for (size_t i = 0; i < OUTPUT_CHANNELS; i++) {
			output_data_ref[b * OUTPUT_CHANNELS + i] = 0;
			int32_t acc = (int32_t) bias[i];
			for (size_t j = 0; j < INPUT_CHANNELS; j++) {
				acc += ((int32_t)input_data[b * INPUT_CHANNELS + j]) * (int32_t)weights[i * INPUT_CHANNELS + j];
			}
			float facc = scale[i] * (float)acc;
			output_data_ref[b * OUTPUT_CHANNELS + i] = (int8_t)fmaxf(fminf(facc, 127.0f), -128.0f);
		}
	}
	xnn_operator_t fc_opu = NULL;
	printf("Creating QS8 operator!\n");
	status = xnn_create_fully_connected_nc_qs8_qc8w(
		INPUT_CHANNELS,  // Input size per batch
		OUTPUT_CHANNELS, // Output size per batch
		INPUT_CHANNELS,  // Input stride
		OUTPUT_CHANNELS, // Output stride
		0,			   	// Input zero point
		1.0f,			// Input scale
		scale,	 		// kernel scale vector
		weights,         // Weights matrix
		bias,            // Bias vector
		outzp,			   	// output zero point
		1.0f,         	// Output scale
		minzp,       // Min activation
		maxzp,        // Max activation
		0,               // Flags
		NULL,            // Weights cache
		&fc_opu);
	if (status != xnn_status_success) {
		printf("Failed to create Fully Connected operator, status code: %d\n", status);
		return -1;
	}
	// Reshape the operator
	status = xnn_reshape_fully_connected_nc_qs8_qc8w(fc_opu, BATCH_SIZE, threadpool);
	if (status != xnn_status_success) {
		printf("Failed to reshape Fully Connected operator, status code: %d\n", status);
		xnn_delete_operator(fc_opu);
		return -1;
	}
	// Setup the operator
	status = xnn_setup_fully_connected_nc_qs8_qc8w(fc_opu, input_data, output_data);
	if (status != xnn_status_success) {
		printf("Failed to setup Fully Connected operator, status code: %d\n", status);
		xnn_delete_operator(fc_opu);
		return -1;
	}

	printf("Running operator\n");

	// Calculate number of iterations proportionally based on size
	// Reference: 2048 * 256 = 524288, with 5 iterations
	// For smaller sizes, run more iterations to compensate
	const size_t ref_input_channels = 2048;
	const size_t ref_output_channels = 256;
	const int ref_iterations = 256;
	
	size_t current_size = (size_t)INPUT_CHANNELS * (size_t)OUTPUT_CHANNELS;
	size_t ref_size = ref_input_channels * ref_output_channels;
	
	// Calculate iterations: scale by inverse of size ratio
	int num_iterations = (int)((double)ref_iterations * (double)ref_size / (double)current_size);
	// Ensure at least 1 iteration
	if (num_iterations < 1) {
		num_iterations = 1;
	}
	
	printf("Running %d iterations (size ratio: %zu/%zu = %.2f)\n", 
		num_iterations, ref_size, current_size, (double)ref_size / (double)current_size);
	
	// Allocate array to store timestamps
	unsigned long *clock_counts = (unsigned long *)malloc(num_iterations * sizeof(unsigned long));
	if (!clock_counts) {
		printf("Failed to allocate memory for timestamps\n");
		xnn_delete_operator(fc_opu);
		return -1;
	}
	
	// Calculate when last 80% starts (first 20% are warmup)
	int last_80_start = (int)(num_iterations * 0.2);
	
	// Run the operator
	for (int iter = 0; iter < num_iterations; iter++) {
		// Print label before starting last 80% of measurements
		if (iter == last_80_start) {
			printf("=== Starting measurement window (last 80%% of iterations) ===\n");
		}
		
		unsigned long clock_start = cycle();
		status = xnn_run_operator(fc_opu, threadpool);
		if (status != xnn_status_success) {
			printf("Failed to run Fully Connected operator, status code: %d\n", status);
			free(clock_counts);
			xnn_delete_operator(fc_opu);
			return -1;
		}
		unsigned long clock_end = cycle();
		clock_counts[iter] = clock_end - clock_start;
		
		// Print label after finishing last 80% of measurements
		if (iter == num_iterations - 1) {
			printf("=== Ending measurement window ===\n");
		}
	}
	
	// Print only the last 20 iterations to reduce output
	int print_start = (num_iterations > 20) ? (num_iterations - 20) : 0;
	int print_count = num_iterations - print_start;
	
	if (print_start > 0) {
		printf("Clocks taken (showing last %d of %d iterations):\n", print_count, num_iterations);
	} else {
		printf("Clocks taken for all iterations:\n");
	}
	
	for (int iter = print_start; iter < num_iterations; iter++) {
		printf("Clocks taken (%d): %lu\n", iter, clock_counts[iter]);
	}
	
	free(clock_counts);
	

	// Verify the output
	for (size_t b = 0; b < BATCH_SIZE; b++) {
		for (size_t i = 0; i < OUTPUT_CHANNELS; i++) {
			int8_t diff = output_data[b * OUTPUT_CHANNELS + i] - output_data_ref[b * OUTPUT_CHANNELS + i];
			if (diff != 0) {
				printf("failed verification at index %zu, batch %zu: expected %d, got %d\n", i, b,
					output_data_ref[b * OUTPUT_CHANNELS + i], output_data[b * OUTPUT_CHANNELS + i]);
				
					// printf("opu:\n");
					// for (size_t bb = 0; bb < BATCH_SIZE; bb++) {
					// 	for (size_t ii = 0; ii < OUTPUT_CHANNELS; ii++) {
					// 		printf("%d ", output_data[bb * OUTPUT_CHANNELS + ii]);
					// 	}
					// 	printf("\n");
					// }
					// printf("reference:\n");
					// for (size_t bb = 0; bb < BATCH_SIZE; bb++) {
					// 	for (size_t ii = 0; ii < OUTPUT_CHANNELS; ii++) {
					// 		printf("%d ", output_data_ref[bb * OUTPUT_CHANNELS + ii]);
					// 	}
					// 	printf("\n");
					// }
					xnn_delete_operator(fc_opu);
					sys_reboot(SYS_REBOOT_COLD);
					return -1;
			}
		}
	}
	printf("passed verification!\n");

	xnn_delete_operator(fc_opu);
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}