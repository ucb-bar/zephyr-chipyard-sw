#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * tinympc_solve
 * @param x_in   pointer to state vector array; length STATE_LEN
 * @param u_out  pointer to output array; length U_LEN
 *
 * Implement this wrapper to call TinyMPC solver (tiny_init/tiny_solve etc.)
 * or for now return dummy safety outputs.
 */
#define STATE_LEN 12   
#define U_LEN 4      

void tinympc_solve(const float *x_in, float *u_out);

#ifdef __cplusplus
}
#endif
