/****************************************************************************
 * arch/xtensa/src/common/xtensa_window_vector.S
 *
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2016 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2006-2015 Cadence Design Systems Inc.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

	.file	"xtensa_window_vector.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/xtensa/xtensa_specregs.h>
#include <arch/xtensa/core.h>
#include <arch/chip/core-isa.h>

/****************************************************************************
 * Window Vectors
 ****************************************************************************/

/* WINDOW OVERFLOW AND UNDERFLOW EXCEPTION VECTORS AND ALLOCA EXCEPTION HANDLER
 *
 * Here is the code for each window overflow/underflow exception vector and
 * (interspersed) efficient code for handling the alloca exception cause.
 * Window exceptions are handled entirely in the vector area and are very
 * tight for performance. The alloca exception is also handled entirely in
 * the window vector area so comes at essentially no cost in code size.
 * Users should never need to modify them and Cadence Design Systems recommends
 * they do not.
 *
 * Window handlers go at predetermined vector locations according to the
 * Xtensa hardware configuration, which is ensured by their placement in a
 * special section known to the Xtensa linker support package (LSP). Since
 * their offsets in that section are always the same, the LSPs do not define
 * a section per vector.
 *
 * These things are coded for XEA2 only (XEA1 is not supported).
 *
 * Note on Underflow Handlers:
 * The underflow handler for returning from call[i+1] to call[i]
 * must preserve all the registers from call[i+1]'s window.
 * In particular, a0 and a1 must be preserved because the RETW instruction
 * will be reexecuted (and may even underflow if an intervening exception
 * has flushed call[i]'s registers).
 * Registers a2 and up may contain return values.
 */

#if XCHAL_HAVE_WINDOWED

	.section .window_vectors.text, "ax"

/* Window Overflow Exception for Call4.
 *
 * Invoked if a call[i] referenced a register (a4-a15) that contains data from
 * ancestor call[j]; call[j] had done a call4 to call[j+1].
 *
 * On entry here:
 *   window rotated to call[j] start point;
 *   a0-a3 are registers to be saved;
 *   a4-a15 must be preserved;
 *   a5 is call[j+1]'s stack pointer.
 */

	.org	0x0
	.global	_window_overflow4
_window_overflow4:

	s32e	a0, a5, -16		/* Save a0 to call[j+1]'s stack frame */
	s32e	a1, a5, -12		/* Save a1 to call[j+1]'s stack frame */
	s32e	a2, a5,  -8		/* Save a2 to call[j+1]'s stack frame */
	s32e	a3, a5,  -4		/* Save a3 to call[j+1]'s stack frame */
#ifdef CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS
	j   	_overflow4_exit_hook
#else
	rfwo					/* Rotates back to call[i] position */
#endif

/* Window Underflow Exception for Call4
 *
 * Invoked by RETW returning from call[i+1] to call[i] where call[i]'s
 * registers must be reloaded (not live in ARs); where call[i] had done a
 * call4 to call[i+1].
 *
 * On entry here:
 *   window rotated to call[i] start point;
 *   a0-a3 are undefined, must be reloaded with call[i].reg[0..3];
 *   a4-a15 must be preserved (they are call[i+1].reg[0..11]);
 *   a5 is call[i+1]'s stack pointer.
 */

	.org	0x40
	.global	_window_underflow4
_window_underflow4:

	l32e	a0, a5, -16		/* Restore a0 from call[i+1]'s stack frame */
	l32e	a1, a5, -12		/* Restore a1 from call[i+1]'s stack frame */
	l32e	a2, a5,  -8		/* Restore a2 from call[i+1]'s stack frame */
	l32e	a3, a5,  -4		/* Restore a3 from call[i+1]'s stack frame */
#ifdef CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS
	j   	_underflow4_exit_hook
#else
	rfwu
#endif

/* Handle alloca exception generated by interruptee executing 'movsp'.
 * This uses space between the window vectors, so is essentially "free".
 * All interruptee's regs are intact except a0 which is saved in EXCSAVE_1,
 * and PS.EXCM has been set by the exception hardware (can't be interrupted).
 * The fact the alloca exception was taken means the registers associated with
 * the base-save area have been spilled and will be restored by the underflow
 * handler, so those 4 registers are available for scratch.
 * The code is optimized to avoid unaligned branches and minimize cache misses.
 */

	.align	4
	.global	_xtensa_alloca_handler
_xtensa_alloca_handler:

	rsr		a0, WINDOWBASE	/* Grab WINDOWBASE before rotw changes it */
	rotw	-1				/* WINDOWBASE goes to a4, new a0-a3 are scratch */
	rsr		a2, PS
	extui	a3, a2, XCHAL_PS_OWB_SHIFT, XCHAL_PS_OWB_BITS
	xor		a3, a3, a4		/* Bits changed from old to current windowbase */
	rsr		a4, EXCSAVE_1	/* Restore original a0 (now in a4) */
	slli	a3, a3, XCHAL_PS_OWB_SHIFT
	xor		a2, a2, a3		/* Flip changed bits in old window base */
	wsr		a2, PS			/* Update PS.OWB to new window base */
	rsync

	_bbci.l	a4, 31, _window_underflow4
	rotw	-1				/* Original a0 goes to a8 */
	_bbci.l	a8, 30, _window_underflow8
	rotw	-1
	j		_window_underflow12

/* Window Overflow Exception for Call8
 *
 * Invoked if a call[i] referenced a register (a4-a15) that contains data from
 * ancestor call[j]; call[j] had done a call8 to call[j+1].
 *
 * On entry here:
 *   window rotated to call[j] start point;
 *   a0-a7 are registers to be saved;
 *   a8-a15 must be preserved;
 *   a9 is call[j+1]'s stack pointer.
 */

	.org	0x80
	.global	_window_overflow8
_window_overflow8:

	s32e	a0, a9, -16		/* Save a0 to call[j+1]'s stack frame */
	l32e	a0, a1, -12		/* a0 <- call[j-1]'s sp
							 * (used to find end of call[j]'s frame) */
	s32e	a1, a9, -12		/* Save a1 to call[j+1]'s stack frame */
	s32e	a2, a9,  -8		/* Save a2 to call[j+1]'s stack frame */
	s32e	a3, a9,  -4		/* Save a3 to call[j+1]'s stack frame */
	s32e	a4, a0, -32		/* Save a4 to call[j]'s stack frame */
	s32e	a5, a0, -28		/* Save a5 to call[j]'s stack frame */
	s32e	a6, a0, -24		/* Save a6 to call[j]'s stack frame */
	s32e	a7, a0, -20		/* Save a7 to call[j]'s stack frame */
#ifdef CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS
	j   	_overflow8_exit_hook
#else
	rfwo					/* Rotates back to call[i] position */
#endif

/* Window Underflow Exception for Call8
 *
 * Invoked by RETW returning from call[i+1] to call[i] where call[i]'s
 * registers must be reloaded (not live in ARs); where call[i] had done a
 * call8 to call[i+1].
 *
 * On entry here:
 *   window rotated to call[i] start point;
 *   a0-a7 are undefined, must be reloaded with call[i].reg[0..7];
 *   a8-a15 must be preserved (they are call[i+1].reg[0..7]);
 *   a9 is call[i+1]'s stack pointer.
 */

	.org	0xc0
	.global	_window_underflow8
_window_underflow8:

	l32e	a0, a9, -16		/* Restore a0 from call[i+1]'s stack frame */
	l32e	a1, a9, -12		/* Restore a1 from call[i+1]'s stack frame */
	l32e	a2, a9,  -8		/* Restore a2 from call[i+1]'s stack frame */
	l32e	a7, a1, -12		/* a7 <- call[i-1]'s sp
							 * (used to find end of call[i]'s frame) */
	l32e	a3, a9,  -4		/* Restore a3 from call[i+1]'s stack frame */
	l32e	a4, a7, -32		/* Restore a4 from call[i]'s stack frame */
	l32e	a5, a7, -28		/* Restore a5 from call[i]'s stack frame */
	l32e	a6, a7, -24		/* Restore a6 from call[i]'s stack frame */
	l32e	a7, a7, -20		/* Restore a7 from call[i]'s stack frame */
#ifdef CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS
	j   	_underflow8_exit_hook
#else
	rfwu
#endif

/* Window Overflow Exception for Call12
 *
 * Invoked if a call[i] referenced a register (a4-a15) that contains data
 * from ancestor call[j]; call[j] had done a call12 to call[j+1].
 *
 * On entry here:
 *   window rotated to call[j] start point;
 *   a0-a11 are registers to be saved;
 *   a12-a15 must be preserved;
 *   a13 is call[j+1]'s stack pointer.
 */

	.org	0x100
	.global	_window_overflow12
_window_overflow12:

	s32e	a0,  a13, -16	/* Save a0 to call[j+1]'s stack frame */
	l32e	a0,  a1,  -12	/* a0 <- call[j-1]'s sp
							 * (used to find end of call[j]'s frame) */
	s32e	a1,  a13, -12	/* Save a1 to call[j+1]'s stack frame */
	s32e	a2,  a13,  -8	/* Save a2 to call[j+1]'s stack frame */
	s32e	a3,  a13,  -4	/* Save a3 to call[j+1]'s stack frame */
	s32e	a4,  a0,  -48	/* Save a4 to end of call[j]'s stack frame */
	s32e	a5,  a0,  -44	/* Save a5 to end of call[j]'s stack frame */
	s32e	a6,  a0,  -40	/* Save a6 to end of call[j]'s stack frame */
	s32e	a7,  a0,  -36	/* Save a7 to end of call[j]'s stack frame */
	s32e	a8,  a0,  -32	/* Save a8 to end of call[j]'s stack frame */
	s32e	a9,  a0,  -28	/* Save a9 to end of call[j]'s stack frame */
	s32e	a10, a0,  -24	/* Save a10 to end of call[j]'s stack frame */
	s32e	a11, a0,  -20	/* Save a11 to end of call[j]'s stack frame */
	rfwo					/* Rotates back to call[i] position */

/* Window Underflow Exception for Call12
 *
 * Invoked by RETW returning from call[i+1] to call[i] where call[i]'s
 * registers must be reloaded (not live in ARs); where call[i] had done a
 * call12 to call[i+1].
 *
 * On entry here:
 *   window rotated to call[i] start point;
 *   a0-a11 are undefined, must be reloaded with call[i].reg[0..11];
 *   a12-a15 must be preserved (they are call[i+1].reg[0..3]);
 *   a13 is call[i+1]'s stack pointer.
 */

	.org 0x140
	.global	_window_underflow12
_window_underflow12:

	l32e	a0,  a13, -16	/* Restore a0 from call[i+1]'s stack frame */
	l32e	a1,  a13, -12	/* Restore a1 from call[i+1]'s stack frame */
	l32e	a2,  a13,  -8	/* Restore a2 from call[i+1]'s stack frame */
	l32e	a11, a1,  -12	/* a11 <- call[i-1]'s sp
							 * (used to find end of call[i]'s frame) */
	l32e	a3,  a13,  -4	/* Restore a3 from call[i+1]'s stack frame */
	l32e	a4,  a11, -48	/* Restore a4 from end of call[i]'s stack frame */
	l32e	a5,  a11, -44	/* Restore a5 from end of call[i]'s stack frame */
	l32e	a6,  a11, -40	/* Restore a6 from end of call[i]'s stack frame */
	l32e	a7,  a11, -36	/* Restore a7 from end of call[i]'s stack frame */
	l32e	a8,  a11, -32	/* Restore a8 from end of call[i]'s stack frame */
	l32e	a9,  a11, -28	/* Restore a9 from end of call[i]'s stack frame */
	l32e	a10, a11, -24	/* Restore a10 from end of call[i]'s stack frame */
	l32e	a11, a11, -20	/* Restore a11 from end of call[i]'s stack frame */
	rfwu

#endif /* XCHAL_HAVE_WINDOWED */
