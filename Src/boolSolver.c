/*
 * boolSolver.c
 *
 *  Created on: 18.02.2018
 *      Author: Micha³ Kuzemczak
 */
#include "boolSolver.h"
#include "uartCom.h"

int solveBool(char *statement, char *inputs) {
	char rpn[100];
	char stack[100];
	char el;
	int rpn_cntr = 0;
	int stack_cntr = 0;
	int i0, i1;
	standardToRPN(statement, rpn, inputs);
	while(rpn[rpn_cntr] != 'A') {
		el = rpn[rpn_cntr++];
		if (el == 1 || el == 0) {
			stack[stack_cntr++] = el;
		} else {
			i0 = (int)stack[--stack_cntr];
			i1 = (int)stack[--stack_cntr];
			if(el == '*' || el == '&') {
				if(i0 && i1) stack[stack_cntr++] = 1;
				else stack[stack_cntr++] = 0;
			} else if (el == '+' || el == '|') {
				if(i0 || i1) stack[stack_cntr++] = 1;
				else stack[stack_cntr++] = 0;
			}
		}
	}
	return (int)stack[--stack_cntr];
}

void standardToRPN(char *in, char *out, char* inputs) {
	char stack[100];
	char el;
	int in_cntr = 0;
	int out_cntr = 0;
	int stack_cntr = 0;
	while (in[in_cntr] != '\0') {
		el = in[in_cntr++];
		if(el != ' ' && el != 'i' && el != 'I') {
			if(el >= '0' && el <= '9') {
				out[out_cntr++] = inputs[el - '0'];
				out[out_cntr] = 'A';
			} else if (el == '(') {
				stack[stack_cntr++] = '(';
			} else if (el == ')') {
				while(stack[--stack_cntr] != '(' && stack_cntr >= 0) {
					out[out_cntr++] = stack[stack_cntr];
					out[out_cntr] = 'A';
				}
			} else if (el == '+' || el == '|') {
				while (stack[stack_cntr - 1] != '(' && (stack_cntr - 1) >= 0) {
					out[out_cntr++] = stack[--stack_cntr];
					out[out_cntr] = 'A';
				}
				if(stack[stack_cntr - 1] == '(') stack_cntr--;
				stack[stack_cntr++] = el;
			} else if (el == '*' || el == '&') {
				while ((stack[stack_cntr - 1] == '*' || stack[stack_cntr - 1] == '&') && stack_cntr > 0) {
					--stack_cntr;
					out[out_cntr++] = stack[stack_cntr];
					out[out_cntr] = 'A';
				}
				stack[stack_cntr++] = el;
			}
		}
	}
	while (stack_cntr-- > 0) {
		out[out_cntr++] = stack[stack_cntr];
		out[out_cntr] = 'A';
	}
}
